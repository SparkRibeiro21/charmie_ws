#pragma once

#include <mutex>
#include <atomic>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "charmie_interfaces/msg/radar_data.hpp"
#include "charmie_interfaces/msg/sdnl_debug.hpp"
#include "charmie_interfaces/action/navigate_sdnl.hpp"

#include "charmie_sdnl_nav/sdnl_core.hpp"
#include "charmie_sdnl_nav/debug_builder.hpp"

class SDNLNavNode : public rclcpp::Node
{
public:
  using NavigateSDNL = charmie_interfaces::action::NavigateSDNL;
  using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateSDNL>;

  SDNLNavNode();

private:
  // Action handlers
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const NavigateSDNL::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigate> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle);

  // Sub callbacks
  void poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
  void radarCallback(const charmie_interfaces::msg::RadarData::SharedPtr msg);

  // Timers
  void controlLoop();
  void debugLoop();

  void publishZeroCmd();

  // Params
  double control_rate_hz_;
  bool debug_enabled_;
  double debug_rate_hz_;
  int n_samples_;

  std::string topic_robot_localisation_;
  std::string topic_radar_data_;
  std::string topic_cmd_vel_;
  std::string topic_sdnl_debug_;

  double robot_radius_;
  double default_max_linear_speed_;
  double default_max_angular_speed_;
  double default_reached_radius_;
  double default_yaw_tolerance_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
  rclcpp::Subscription<charmie_interfaces::msg::RadarData>::SharedPtr radar_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<charmie_interfaces::msg::SDNLDebug>::SharedPtr debug_pub_;

  rclcpp_action::Server<NavigateSDNL>::SharedPtr action_server_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  // Cached data
  std::mutex data_mutex_;
  geometry_msgs::msg::Pose2D last_pose_;
  charmie_interfaces::msg::RadarData last_radar_;
  bool have_pose_;
  bool have_radar_;

  // Goal state
  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandleNavigate> active_goal_handle_;
  NavigateSDNL::Goal active_goal_;
  std::atomic<bool> goal_active_;
  std::atomic<bool> cancel_requested_;

  // SDNL core + debug
  sdnl::SdnlCore core_;
  sdnl::DebugBuilder debug_builder_;

  // Latest computed for debug
  std::mutex debug_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  double latest_dist_;
  double latest_yaw_err_;
  double latest_min_obs_edge_;
  double latest_psi_target_base_;
};
