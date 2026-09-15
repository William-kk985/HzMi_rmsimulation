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

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

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
    xyz_ = this->declare_parameter<std::vector<double>>("xyz", {0.0, 0.0, 0.0});
    rpy_ = this->declare_parameter<std::vector<double>>("rpy", {0.0, 0.0, 0.0});

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LioTfAdapter::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "lio_tf_adapter 启动：订阅 %s -> 广播 TF %s -> %s",
      odom_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
  }

private:
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

    tf_broadcaster_->sendTransform(tf_msg);
  }

  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::vector<double> xyz_;
  std::vector<double> rpy_;

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
