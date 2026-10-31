// =============================================================================
// mecanum_drive_plugin.cpp —— 起伏底盘（4 轮麦轮）的自定义 Gazebo 驱动插件
//
// 为什么不用 ros2_control：实测 gazebo_ros2_control 从 robot_state_publisher 参数服务器
// 取 robot_description 时解析失败（parser error --param robot_description:=<?xml ...），
// 控制器管理器起不来。本插件直接读 /cmd_vel_chassis 做麦轮逆解，绕开整条 ros2_control。
//
// 输入：/cmd_vel_chassis (geometry_msgs/Twist)  vx, vy, wz
// 输出：4 个轮 joint 的 velocity + /odom_ground_truth (nav_msgs/Odometry, 模型世界位姿)
//
// 麦轮逆解（r=轮半径, s=lx+ly）：
//   FL = (vx - vy - s*wz)/r
//   FR = (vx + vy + s*wz)/r
//   RL = (vx + vy - s*wz)/r
//   RR = (vx - vy + s*wz)/r
// 4 个轮轴统一 +y（麦轮 4 轮同向滚，不是差速的左右反号）。
// =============================================================================
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <cmath>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace chassis_sim
{

class MecanumDrivePlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    radius_ = sdf->Get<double>("wheel_radius", 0.058).first;
    sum_lx_ly_ = sdf->Get<double>("wheel_separation_sum", 0.302).first;
    timeout_ = sdf->Get<double>("cmd_timeout", 0.5).first;
    max_wheel_speed_ = sdf->Get<double>("max_wheel_speed", 40.0).first;
    // ★ 仿真标定：实测"指令角速度 → 实际 yaw"约 0.2 倍（轮地接触侧向阻力），
    //   这里补偿 5 倍，使 nav2 的角速度指令 ≈ 车体实际角速度（闭环仍走 LIO）
    yaw_gain_ = sdf->Get<double>("yaw_gain", 1.0).first;

    const char * names[4] = {"w1_joint", "w2_joint", "w3_joint", "w4_joint"};
    for (int i = 0; i < 4; ++i) joint_names_[i] = names[i];
    for (int i = 0; i < 4; ++i) {
      auto j = model_->GetJoint(names[i]);
      if (!j) {
        gzerr << "[mecanum_drive] 找不到关节 " << names[i] << "\n";
        return;
      }
      joints_[i] = j;
    }

    ros_node_ = gazebo_ros::Node::Get(sdf);
    sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_chassis", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_ = *msg;
        last_cmd_time_ = model_->GetWorld()->SimTime();
        have_cmd_ = true;
      });
    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/odom_ground_truth", 10);

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&MecanumDrivePlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(),
      "[mecanum_drive] 就绪：r=%.4f  timeout=%.2fs", radius_, timeout_);
  }

private:
  void OnUpdate()
  {
    const auto sim_time = model_->GetWorld()->SimTime();
    geometry_msgs::msg::Twist cmd;
    if (have_cmd_ && (sim_time - last_cmd_time_).Double() < timeout_) {
      cmd = cmd_;
    }
    // ★ 2026-10-31 修正转向符号 + 用**每个轮子的真实 y**（URDF 实测：
    //   w1(FL)=(+0.1615,-0.10895) w2(FR)=(+0.1615,+0.17205)
    //   w3(RL)=(-0.1615,-0.17205) w4(RR)=(-0.1615,+0.10895)）
    //   刚体：轮心 x 向速度 = vx - wz*y_i  ⇒ ω_i = (vx - wz*y_i + vy*s_vy_i)/r
    //   （旧实现用 ±(lx+ly) 统一符号，把左右轮的 yaw 项写成反的 ⇒ 车往反方向转）
    const double r = radius_;
    const double y[4] = {-0.10895, 0.17205, -0.17205, 0.10895};
    const double s_vy[4] = {-1.0, 1.0, 1.0, -1.0};   // 麦轮横移符号（45° 辊子对角）
    double v[4];
    for (int i = 0; i < 4; ++i) {
      v[i] = (cmd.linear.x - cmd.angular.z * yaw_gain_ * y[i] + cmd.linear.y * s_vy[i]) / r;
    }
    for (int i = 0; i < 4; ++i) {
      if (v[i] > max_wheel_speed_) v[i] = max_wheel_speed_;
      if (v[i] < -max_wheel_speed_) v[i] = -max_wheel_speed_;
      joints_[i]->SetVelocity(0, v[i]);   // 物理轮驱动（IMU 才有加速度 → LIO 才动）
    }

    // ★ 2026-10-31 回到**物理轮驱动**：运动学 SetLinearVel/SetAngularVel 会让 Gazebo 的 IMU
    //   看不到加速度 ⇒ LIO 里程计原地不动（实测 LIO 位移 0.00 m，而 GT 走了 4.47 m）
    //   ⇒ 导航姿态估计全靠 gicp，朝向错 ⇒ 车斜着跑。轮子转起来，IMU 才有真实加速度。

    // 诊断：每 0.5 s 打印一次"目标 vs 实际"轮速（确认 SetVelocity 是否生效）
    if ((sim_time - last_dbg_time_).Double() > 0.5) {
      last_dbg_time_ = sim_time;
      RCLCPP_INFO(ros_node_->get_logger(),
        "[mecanum_drive] cmd(vx=%.3f wz=%.3f) target=[%.2f,%.2f,%.2f,%.2f] actual=[%.2f,%.2f,%.2f,%.2f]",
        cmd.linear.x, cmd.angular.z, v[0], v[1], v[2], v[3],
        joints_[0]->GetVelocity(0), joints_[1]->GetVelocity(0),
        joints_[2]->GetVelocity(0), joints_[3]->GetVelocity(0));
    }

    // 里程计：模型世界位姿（GT）—— 限频 ~50 Hz（原来每个物理步都发 1 kHz）
    if ((sim_time - last_odom_time_).Double() < 0.02) return;
    last_odom_time_ = sim_time;
    const auto pose = model_->WorldPose();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = ros_node_->get_clock()->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = cmd.linear.x;
    odom.twist.twist.linear.y = cmd.linear.y;
    odom.twist.twist.angular.z = cmd.angular.z;
    odom_pub_->publish(odom);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joints_[4];
  gazebo::event::ConnectionPtr update_conn_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::string joint_names_[4];
  double radius_ = 0.058, sum_lx_ly_ = 0.302, timeout_ = 0.5, max_wheel_speed_ = 40.0;
  double yaw_gain_ = 1.0;
  geometry_msgs::msg::Twist cmd_;
  gazebo::common::Time last_cmd_time_;
  gazebo::common::Time last_odom_time_;
  gazebo::common::Time last_dbg_time_;
  bool have_cmd_ = false;
};

}  // namespace chassis_sim

GZ_REGISTER_MODEL_PLUGIN(chassis_sim::MecanumDrivePlugin)
