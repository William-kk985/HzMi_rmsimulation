#include "rm_goal_checker/plan_goal_checker.hpp"

#include <cmath>
#include <limits>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace rm_goal_checker
{

void PlanGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = plugin_name;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("PlanGoalChecker: parent node 已失效");
  }
  lc_node_ = node;
  tf_ = costmap_ros->getTfBuffer();
  robot_frame_ = costmap_ros->getBaseFrameID();

  lc_node_->declare_parameter(plugin_name_ + ".xy_goal_tolerance", 0.25);
  lc_node_->declare_parameter(plugin_name_ + ".yaw_goal_tolerance", 0.25);
  lc_node_->get_parameter(plugin_name_ + ".xy_goal_tolerance", xy_tolerance_);
  lc_node_->get_parameter(plugin_name_ + ".yaw_goal_tolerance", yaw_tolerance_);

  plan_sub_ = lc_node_->create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(1),
    [this](nav_msgs::msg::Path::SharedPtr msg) {
      std::lock_guard<std::mutex> lk(plan_mutex_);
      plan_ = msg;
    });

  RCLCPP_INFO(
    lc_node_->get_logger(),
    "[plan_goal_checker] 就绪：robot_frame=%s xy_tol=%.3f yaw_tol=%.3f（目标取自 /plan 末点）",
    robot_frame_.c_str(), xy_tolerance_, yaw_tolerance_);
}

void PlanGoalChecker::reset()
{
  // 无锁存状态：每次都用最新 /plan 末点判定
}

bool PlanGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & /*query_pose*/,
  const geometry_msgs::msg::Pose & /*goal_pose*/,
  const geometry_msgs::msg::Twist & /*velocity*/)
{
  nav_msgs::msg::Path::SharedPtr plan;
  {
    std::lock_guard<std::mutex> lk(plan_mutex_);
    plan = plan_;
  }
  if (!plan || plan->poses.empty()) {
    return false;
  }

  const auto & goal_stamped = plan->poses.back();
  const std::string goal_frame =
    goal_stamped.header.frame_id.empty() ? plan->header.frame_id : goal_stamped.header.frame_id;
  if (goal_frame.empty()) {
    return false;
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    // ★ TimePointZero = 用**最新**变换，不挑时间戳 ⇒ 不会再因"时间戳太旧"失败
    tf_msg = tf_->lookupTransform(goal_frame, robot_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      lc_node_->get_logger(), *lc_node_->get_clock(), 2000,
      "[plan_goal_checker] 查 %s→%s 失败：%s", goal_frame.c_str(), robot_frame_.c_str(), ex.what());
    return false;
  }

  geometry_msgs::msg::PoseStamped robot_in_robot;
  robot_in_robot.header.frame_id = robot_frame_;
  robot_in_robot.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped robot_in_goal;
  tf2::doTransform(robot_in_robot, robot_in_goal, tf_msg);

  const double dx = robot_in_goal.pose.position.x - goal_stamped.pose.position.x;
  const double dy = robot_in_goal.pose.position.y - goal_stamped.pose.position.y;
  const double dist = std::hypot(dx, dy);
  const double dyaw = std::fabs(
    tf2::getYaw(robot_in_goal.pose.orientation) - tf2::getYaw(goal_stamped.pose.orientation));

  if (dist > xy_tolerance_) {
    return false;
  }
  return dyaw <= yaw_tolerance_;
}

bool PlanGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  pose_tolerance.position.x = xy_tolerance_;
  pose_tolerance.position.y = xy_tolerance_;
  pose_tolerance.position.z = std::numeric_limits<double>::lowest();
  vel_tolerance.linear.x = std::numeric_limits<double>::lowest();
  vel_tolerance.linear.y = std::numeric_limits<double>::lowest();
  vel_tolerance.linear.z = std::numeric_limits<double>::lowest();
  vel_tolerance.angular.x = std::numeric_limits<double>::lowest();
  vel_tolerance.angular.y = std::numeric_limits<double>::lowest();
  vel_tolerance.angular.z = std::numeric_limits<double>::lowest();
  return true;
}

}  // namespace rm_goal_checker

PLUGINLIB_EXPORT_CLASS(rm_goal_checker::PlanGoalChecker, nav2_core::GoalChecker)
