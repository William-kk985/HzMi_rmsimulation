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
    js_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&MecanumDrivePlugin::OnUpdate, this));

    gzmsg << "[mecanum_drive] 就绪：r=" << radius_ << " lx+ly=" << sum_lx_ly_
          << " timeout=" << timeout_ << "s\n";
  }

private:
  void OnUpdate()
  {
    const auto sim_time = model_->GetWorld()->SimTime();
    geometry_msgs::msg::Twist cmd;
    if (have_cmd_ && (sim_time - last_cmd_time_).Double() < timeout_) {
      cmd = cmd_;
    }
    const double r = radius_;
    const double s = sum_lx_ly_;
    double v[4];
    v[0] = (cmd.linear.x - cmd.linear.y - s * cmd.angular.z) / r;
    v[1] = (cmd.linear.x + cmd.linear.y + s * cmd.angular.z) / r;
    v[2] = (cmd.linear.x + cmd.linear.y - s * cmd.angular.z) / r;
    v[3] = (cmd.linear.x - cmd.linear.y + s * cmd.angular.z) / r;
    for (int i = 0; i < 4; ++i) {
      if (v[i] > max_wheel_speed_) v[i] = max_wheel_speed_;
      if (v[i] < -max_wheel_speed_) v[i] = -max_wheel_speed_;
      joints_[i]->SetVelocity(0, v[i]);
    }

    // 里程计：模型世界位姿（GT）
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

    // 关节状态（让 RViz 里的轮子真的跟着转；joint_state_publisher 在 chassis 下已被让位）
    sensor_msgs::msg::JointState js;
    js.header.stamp = odom.header.stamp;
    for (int i = 0; i < 4; ++i) {
      js.name.emplace_back(joint_names_[i]);
      js.position.push_back(joints_[i]->Position(0));
      js.velocity.push_back(joints_[i]->GetVelocity(0));
    }
    js_pub_->publish(js);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr joints_[4];
  gazebo::event::ConnectionPtr update_conn_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  std::string joint_names_[4];
  double radius_ = 0.058, sum_lx_ly_ = 0.302, timeout_ = 0.5, max_wheel_speed_ = 40.0;
  geometry_msgs::msg::Twist cmd_;
  gazebo::common::Time last_cmd_time_;
  bool have_cmd_ = false;
};

}  // namespace chassis_sim

GZ_REGISTER_MODEL_PLUGIN(chassis_sim::MecanumDrivePlugin)
