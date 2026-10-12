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
#include "charmie_interfaces/msg/sdnl_marker_debug.hpp"
#include "charmie_interfaces/action/navigate_sdnl.hpp"

#include "charmie_sdnl_nav/sdnl_core.hpp"

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

  void publishMarkerDebug(bool active, const NavigateSDNL::Goal* goal_ptr = nullptr);

  // Timers
  void controlLoop();
  void debugLoop();

  void publishZeroCmd();

  // Params
  double control_rate_hz_;
  bool debug_enabled_;
  double debug_rate_hz_;
  int n_samples_;
  double marker_debug_rate_hz_;
  double radar_timeout_s_;

  std::string topic_robot_localisation_;
  std::string topic_radar_data_;
  std::string topic_cmd_vel_;
  std::string topic_sdnl_debug_;
  std::string topic_sdnl_marker_debug_;

  double default_max_linear_speed_;
  double default_max_angular_speed_;
  double default_reached_radius_;
  double default_yaw_tolerance_;

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_;
  rclcpp::Subscription<charmie_interfaces::msg::RadarData>::SharedPtr radar_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<charmie_interfaces::msg::SDNLDebug>::SharedPtr debug_pub_;
  rclcpp::Publisher<charmie_interfaces::msg::SDNLMarkerDebug>::SharedPtr marker_debug_pub_;

  rclcpp_action::Server<NavigateSDNL>::SharedPtr action_server_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  rclcpp::TimerBase::SharedPtr marker_debug_timer_;

  // Stop ticks management, after movement and before final orientation
  int stop_ticks_remaining_{0};
  int stop_ticks_default_{3};

  // Final stop ticks management, after goal completion
  int final_stop_ticks_remaining_{0};
  int final_stop_ticks_default_{6};

  // Timing debug
  bool timing_debug_enabled_;
  int timing_debug_period_ms_;

  // Cached data
  std::mutex data_mutex_;
  geometry_msgs::msg::Pose2D last_pose_;
  bool have_pose_;
  charmie_interfaces::msg::RadarData last_radar_;
  bool have_radar_;
  rclcpp::Time last_radar_stamp_;

  // Goal state
  std::mutex goal_mutex_;
  std::shared_ptr<GoalHandleNavigate> active_goal_handle_;
  NavigateSDNL::Goal active_goal_;
  std::atomic<bool> goal_active_;
  std::atomic<bool> cancel_requested_;

  // Execution phase 
  enum class ExecPhase : uint8_t {
    ROTATE_TO_TARGET_BEARING = 0, // first_rotate (depends of first_rotate flag)
    MOVE_TO_POSITION         = 1,
    STOP_BEFORE_FINAL_ORIENT = 2, // stop and wait before final orientation
    FINAL_ORIENT             = 3, // defined final orientation (depends of orientate_after_move flag)
    FINAL_STOP               = 4  // publish zero cmd_vel for N cycles before succeeding
  };

  std::mutex phase_mutex_;
  ExecPhase exec_phase_{ExecPhase::MOVE_TO_POSITION};

  // SDNL core
  sdnl::SdnlCore core_;

  // Latest debug marker msgs
  charmie_interfaces::msg::SDNLMarkerDebug last_marker_debug_;
  std::mutex marker_debug_mutex_;

  // Latest computed for debug
  std::mutex debug_mutex_;
  geometry_msgs::msg::Twist latest_cmd_;
  geometry_msgs::msg::Pose2D latest_pose_for_debug_;
  NavigateSDNL::Goal latest_goal_for_debug_;
  bool latest_have_pose_for_debug_{false};

  charmie_interfaces::msg::RadarData latest_radar_for_debug_;
  bool latest_have_radar_for_debug_{false};
  rclcpp::Time latest_radar_stamp_for_debug_;

  // Timing / latency metrics
  rclcpp::Time last_goal_accept_time_;
  std::atomic<bool> first_cmd_after_goal_pending_{false};
};
