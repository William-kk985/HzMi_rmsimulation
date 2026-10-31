#ifndef RM_GOAL_CHECKER__PLAN_GOAL_CHECKER_HPP_
#define RM_GOAL_CHECKER__PLAN_GOAL_CHECKER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "nav2_core/goal_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace rm_goal_checker
{

/**
 * @brief 用 /plan 的末点 当目标的 GoalChecker。
 *
 * 为什么不用 nav2 自带的 PositionGoalChecker/SimpleGoalChecker：
 *   nav2_controller::ControllerServer::isGoalReached() 里
 *     nav_2d_utils::transformPose(tf, costmap_frame, end_pose_, transformed_end_pose, tol);
 *   **忽略了返回值**；转换失败时 transformed_end_pose 保持默认 (0,0)，于是
 *   goal_checker 拿车去和 map 原点比 —— 出生点就在原点附近 ⇒ 任何目标都"瞬间到达"。
 *
 * 本插件不依赖那个 goal 参数：自己订阅 /plan（全局规划器输出，frame=map），
 * 取末点，并用 tf2::TimePointZero（**最新**变换，不挑时间戳）把机器人搬到同一系里比。
 */
class PlanGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void reset() override;

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  nav_msgs::msg::Path::SharedPtr plan_;
  std::mutex plan_mutex_;

  std::string plugin_name_;
  std::string robot_frame_;
  double xy_tolerance_{0.25};
  double yaw_tolerance_{0.25};
};

}  // namespace rm_goal_checker

#endif  // RM_GOAL_CHECKER__PLAN_GOAL_CHECKER_HPP_
