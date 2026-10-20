// lio_tf_adapter —— T3：把 LIO 的内部位姿输出翻译成标准导航帧树
//
// 背景：FAST-LIO / Point-LIO 输出的位姿以内部帧 camera_init/body（Point-LIO 为
// camera_init/aft_mapped）表达，而导航栈（Nav2 / AMCL / slam_toolbox / TEB 等）
// 期望标准帧树 map → odom → base_link。
//
// 本节点订阅统一后的里程计话题（默认 /odom，由 bringup 对两套 LIO 做 remap 得到），
// 以标准帧名广播 TF：odom → base_link。这样 LIO 成为导航唯一的里程计来源，
// 仿真与真车一致（不再依赖 Gazebo 真值 TF）。
//
// 可选补偿：LIO 的 body 帧与底盘 base_link 原点可能不重合（默认按重合处理），
// 可通过 xyz/rpy 参数补偿一个小外参。
//
// ============================================================================
// ★ 2026-10-21（契约 v3 §4.11）：**可配置的父帧** + 从帧图取"输入世界 → 父帧"的静态变换
// ----------------------------------------------------------------------------
// 默认路径的帧树变成 `map → odom → odom_level → base_link`：
//   · `odom`      = LIO 的**原始**世界系（IMU 斜装 ⇒ 斜的；E1 默认关闭）；
//   · `odom_level`= 重力对齐（z 朝上）+ 原点在地面；由 `odom_level_tf` 发**一条静态边**
//                   `odom → odom_level`（latched、唯一发布者，值由 URDF+rpy/h 算出）。
// ⇒ 本节点要发的是 `odom_level → base_link`，但它拿到的 `/odom` 是表达在 `odom`（原始世界）
//   里的 T_ol ⇒ 必须**左乘** T_parent←input（= `lookupTransform(odom_frame, input_frame)`）：
//       q_out = q_po * (q_lio * q_offset)
//       t_out = t_po + R_po * (t_lio + R_lio * t_offset)
//   （`parent ← base` = `parent ← input` ∘ `input ← base`，右乘外参那一步与改动前逐字相同。）
//
// 参数语义（**向后兼容**）：
//   · `odom_frame` = 本节点广播的**父帧**（默认 `odom`，= 改动前行为）。默认路径由 launch 传
//                    `odom_level`；回滚档（`level_frame:=odom`）传的值与 YAML 默认值相同 ⇒
//                    运行期参数 dump 与改动前**逐字节相同**，且 `odom_frame == input_frame`
//                    ⇒ 走直通分支、连 tf2 buffer 都不建（graph/数值与改动前逐位相同）。
//   · **输入帧硬编码为 `odom`**（= LIO 的原始世界系；`odom_convention:=sensor` 档的 `/Odometry`
//     frame_id 就是它，`map→odom` 也挂在它上面 —— 这是契约 §2/§4.11 的帧名，不是旋钮）。
//     若父帧 != `odom`，则本节点取 `lookupTransform(odom_frame, "odom")` 并**左乘**。
//   · 查找用 `tf2::TimePointZero`（静态边时间无关），成功一次即缓存；查不到就**不发 TF**并
//     限速报 WARN —— 宁可让 nav2 因缺边而响亮失败，也不发一条错的边。
// ============================================================================

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace lio_tf_adapter
{

class LioTfAdapter : public rclcpp::Node
{
public:
  LioTfAdapter() : Node("lio_tf_adapter")
  {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    // ★ 2026-10-21（§4.11）：输入帧 = 契约里那个**原始世界系**的名字（`odom`）。
    //   不新增参数是有意的：`odom_frame:=odom`（回滚档）时两个名字相同 ⇒ 直通分支，
    //   参数集合与改动前逐字节相同（见文件头）。
    static constexpr const char * kRawWorldFrame = "odom";
    input_frame_ = (odom_frame_ == kRawWorldFrame) ? odom_frame_ : std::string(kRawWorldFrame);
    xyz_ = this->declare_parameter<std::vector<double>>("xyz", {0.0, 0.0, 0.0});
    rpy_ = this->declare_parameter<std::vector<double>>("rpy", {0.0, 0.0, 0.0});
    // ★ 2026-10-20（防御）：odomCallback 里是 `xyz_[0..2]` / `rpy_[0..2]` —— `std::vector::operator[]`
    //   **不查长度**，参数若给了 0/1/2 个数（launch 求值失败、手写 params、`ros2 param set` 写短）
    //   就是越界读堆内存 ⇒ 轻则补偿量是垃圾（车体位置/姿态静默错），重则 **SIGSEGV**。
    //   这里长度不对就 ERROR 一次 + 归零（= 不做补偿，与"没配外参"同义），绝不越界。
    const auto sanitize3 = [this](const char *name, std::vector<double> &v) {
        if (v.size() == 3) {
          return;
        }
        RCLCPP_ERROR(this->get_logger(),
                     "参数 %s 需要 3 个数，收到 %zu 个 ⇒ 按 [0, 0, 0] 处理（不做该项补偿）；"
                     "旧代码这里会越界读 std::vector（值来自堆上邻居字节 ⇒ 静默错补偿，甚至 SIGSEGV）",
                     name, v.size());
        v = {0.0, 0.0, 0.0};
      };
    sanitize3("xyz", xyz_);
    sanitize3("rpy", rpy_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // 只有需要"左乘一条静态边"时才建 buffer/listener（回滚档 graph 上不多一个 /tf 订阅者）
    if (input_frame_ != odom_frame_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    }

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LioTfAdapter::odomCallback, this, std::placeholders::_1));

    if (input_frame_ == odom_frame_) {
      RCLCPP_INFO(
        this->get_logger(), "lio_tf_adapter 启动：订阅 %s -> 广播 TF %s -> %s（父帧=输入帧，直通）",
        odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "lio_tf_adapter 启动：订阅 %s（表达在 %s）-> 广播 TF %s -> %s"
        "（左乘 T_%s←%s 静态边；唯一发布者 = odom_level_tf，查不到就不发）",
        odom_topic_.c_str(), input_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str(),
        odom_frame_.c_str(), input_frame_.c_str());
    }
  }

private:
  // 取（并缓存）T_parent←input。静态边 ⇒ 时间无关；成功一次就不再查。
  // 返回 false = 还没就绪（边不存在 / 尚未到达）⇒ 调用方**不发** TF。
  bool levelTransform(tf2::Transform &out)
  {
    if (level_ready_) {
      out = level_tf_;
      return true;
    }
    if (!tf_buffer_) {
      level_tf_.setIdentity();
      level_ready_ = true;
      out = level_tf_;
      return true;
    }
    try {
      const auto tf = tf_buffer_->lookupTransform(odom_frame_, input_frame_, tf2::TimePointZero);
      const auto &q = tf.transform.rotation;
      const auto &t = tf.transform.translation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      quat.normalize();
      level_tf_.setRotation(quat);
      level_tf_.setOrigin(tf2::Vector3(t.x, t.y, t.z));
      level_ready_ = true;
      out = level_tf_;
      double rr = 0.0, pp = 0.0, yy = 0.0;
      tf2::Matrix3x3(quat).getRPY(rr, pp, yy);
      constexpr double k_r2d = 57.29577951308232;
      RCLCPP_INFO(
        this->get_logger(),
        "lio_tf_adapter：取到静态边 T_%s←%s = xyz[%.6f, %.6f, %.6f] rpy[%.4f, %.4f, %.4f]°"
        "（之后每帧左乘它；与 odom_level_tf 的发布值逐位相同）",
        odom_frame_.c_str(), input_frame_.c_str(), t.x, t.y, t.z,
        rr * k_r2d, pp * k_r2d, yy * k_r2d);
      return true;
    } catch (const tf2::TransformException &ex) {
      if (++level_miss_ % 100 == 1) {
        RCLCPP_WARN(
          this->get_logger(),
          "lio_tf_adapter：查不到 T_%s←%s（%s）⇒ 暂不发 TF（第 %d 次；静态边 owner = "
          "odom_level_tf，确认它在跑）",
          odom_frame_.c_str(), input_frame_.c_str(), ex.what(), level_miss_);
      }
      return false;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = odom_frame_;
    tf_msg.child_frame_id = base_frame_;

    // 位姿四元数（LIO 输出）与可选补偿外参（base_link 相对 body）
    tf2::Quaternion q_lio(
      msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Quaternion q_offset;
    q_offset.setRPY(rpy_[0], rpy_[1], rpy_[2]);

    const tf2::Quaternion q_out = q_lio * q_offset;
    const tf2::Vector3 t_offset(xyz_[0], xyz_[1], xyz_[2]);
    const tf2::Vector3 t_rotated = tf2::quatRotate(q_lio, t_offset);

    tf_msg.transform.translation.x = msg->pose.pose.position.x + t_rotated.x();
    tf_msg.transform.translation.y = msg->pose.pose.position.y + t_rotated.y();
    tf_msg.transform.translation.z = msg->pose.pose.position.z + t_rotated.z();
    // 手动赋值，避免依赖 tf2_geometry_msgs 的 toMsg 特化
    tf_msg.transform.rotation.x = q_out.x();
    tf_msg.transform.rotation.y = q_out.y();
    tf_msg.transform.rotation.z = q_out.z();
    tf_msg.transform.rotation.w = q_out.w();

    // ★ 2026-10-21（§4.11）：把"输入世界 → 父帧"那条静态边**左乘**上来（逐位不变的条件：
    //   父帧 == 输入帧 ⇒ 走直通分支，q/t 与改动前逐位相同）。
    if (input_frame_ != odom_frame_) {
      tf2::Transform t_parent_input;
      if (!levelTransform(t_parent_input)) {
        return;                       // 边还没就绪 ⇒ 不发（绝不发一条错的边）
      }
      tf2::Transform t_input_base;
      t_input_base.setRotation(q_out);
      t_input_base.setOrigin(tf2::Vector3(tf_msg.transform.translation.x,
                                         tf_msg.transform.translation.y,
                                         tf_msg.transform.translation.z));
      const tf2::Transform t_parent_base = t_parent_input * t_input_base;
      tf_msg.transform.translation.x = t_parent_base.getOrigin().x();
      tf_msg.transform.translation.y = t_parent_base.getOrigin().y();
      tf_msg.transform.translation.z = t_parent_base.getOrigin().z();
      const tf2::Quaternion q_pb = t_parent_base.getRotation();
      tf_msg.transform.rotation.x = q_pb.x();
      tf_msg.transform.rotation.y = q_pb.y();
      tf_msg.transform.rotation.z = q_pb.z();
      tf_msg.transform.rotation.w = q_pb.w();
    }

    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string input_frame_;                 // /odom 表达在哪个帧（默认 = odom_frame）
  std::vector<double> xyz_;
  std::vector<double> rpy_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Transform level_tf_;                 // T_parent←input（静态；缓存）
  bool level_ready_ = false;
  int level_miss_ = 0;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

}  // namespace lio_tf_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lio_tf_adapter::LioTfAdapter>());
  rclcpp::shutdown();
  return 0;
}
