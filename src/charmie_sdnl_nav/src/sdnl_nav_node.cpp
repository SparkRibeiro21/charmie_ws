#include "charmie_sdnl_nav/sdnl_nav_node.hpp"
#include "charmie_sdnl_nav/angle_utils.hpp"

#include <chrono>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

SDNLNavNode::SDNLNavNode()
: Node("sdnl_nav"),
  control_rate_hz_(this->declare_parameter<double>("control_rate_hz", 30.0)),
  debug_enabled_(this->declare_parameter<bool>("debug_enabled", false)),
  debug_rate_hz_(this->declare_parameter<double>("debug_rate_hz", 10.0)),
  n_samples_(this->declare_parameter<int>("n_samples", 720)),
  topic_robot_localisation_(this->declare_parameter<std::string>("topic_robot_localisation", "robot_localisation")),
  topic_radar_data_(this->declare_parameter<std::string>("topic_radar_data", "radar/data")),
  topic_cmd_vel_(this->declare_parameter<std::string>("topic_cmd_vel", "cmd_vel")),
  topic_sdnl_debug_(this->declare_parameter<std::string>("topic_sdnl_debug", "sdnl/debug")),
  robot_radius_(this->declare_parameter<double>("robot_radius", 0.28)),
  default_max_linear_speed_(this->declare_parameter<double>("default_max_linear_speed", 0.4)),
  default_max_angular_speed_(this->declare_parameter<double>("default_max_angular_speed", 1.0)),
  default_reached_radius_(this->declare_parameter<double>("default_reached_radius", 0.6)),
  default_yaw_tolerance_(this->declare_parameter<double>("default_yaw_tolerance", 0.26)),
  have_pose_(false),
  have_radar_(false),
  goal_active_(false),
  cancel_requested_(false),
  core_([&]{
    sdnl::SdnlParams p;
    p.lambda_target_not_obs  = this->declare_parameter<double>("lambda_target_not_obs", 10.0);
    p.lambda_target_with_obs = this->declare_parameter<double>("lambda_target_with_obs", 4.0);
    p.beta1                  = this->declare_parameter<double>("beta1", 40.0);
    p.beta2                  = this->declare_parameter<double>("beta2", 0.06);
    p.max_dist_for_obs        = this->declare_parameter<double>("max_dist_for_obs", 2.0);
    p.robot_radius           = this->declare_parameter<double>("robot_radius", 0.28);
    p.heading_offset_rad     = this->declare_parameter<double>("heading_offset_rad", 0.0);
    return sdnl::SdnlCore(p);
  }()),
  debug_builder_(core_),
  latest_dist_(0.0),
  latest_yaw_err_(0.0),
  latest_min_obs_edge_(std::numeric_limits<double>::infinity()),
  latest_psi_target_base_(0.0)
{
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_cmd_vel_, 10);

  if (debug_enabled_) {
    debug_pub_ = this->create_publisher<charmie_interfaces::msg::SDNLDebug>(topic_sdnl_debug_, 10);
  }

  pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
    topic_robot_localisation_, 10,
    std::bind(&SDNLNavNode::poseCallback, this, std::placeholders::_1));

  radar_sub_ = this->create_subscription<charmie_interfaces::msg::RadarData>(
    topic_radar_data_, 10,
    std::bind(&SDNLNavNode::radarCallback, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<NavigateSDNL>(
    this,
    "navigate_sdnl",
    std::bind(&SDNLNavNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SDNLNavNode::handle_cancel, this, std::placeholders::_1),
    std::bind(&SDNLNavNode::handle_accepted, this, std::placeholders::_1));

  const auto control_period = std::chrono::duration<double>(1.0 / std::max(1e-6, control_rate_hz_));
  control_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(control_period),
    std::bind(&SDNLNavNode::controlLoop, this));

  if (debug_enabled_) {
    const auto debug_period = std::chrono::duration<double>(1.0 / std::max(1e-6, debug_rate_hz_));
    debug_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(debug_period),
      std::bind(&SDNLNavNode::debugLoop, this));
  }

  RCLCPP_INFO(this->get_logger(), "SDNLNavNode started (control_rate=%.1f Hz, debug=%s).",
              control_rate_hz_, debug_enabled_ ? "true" : "false");
}

rclcpp_action::GoalResponse SDNLNavNode::handle_goal(
  const rclcpp_action::GoalUUID& /*uuid*/,
  std::shared_ptr<const NavigateSDNL::Goal> /*goal*/)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SDNLNavNode::handle_cancel(
  const std::shared_ptr<GoalHandleNavigate> /*goal_handle*/)
{
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SDNLNavNode::handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  std::lock_guard<std::mutex> lk(goal_mutex_);
  active_goal_handle_ = goal_handle;
  active_goal_ = *(goal_handle->get_goal());
  goal_active_.store(true);
  cancel_requested_.store(false);
}

void SDNLNavNode::poseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  last_pose_ = *msg;
  have_pose_ = true;
}

void SDNLNavNode::radarCallback(const charmie_interfaces::msg::RadarData::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(data_mutex_);
  last_radar_ = *msg;
  have_radar_ = true;
}

void SDNLNavNode::publishZeroCmd()
{
  geometry_msgs::msg::Twist t;
  cmd_vel_pub_->publish(t);
}

void SDNLNavNode::controlLoop()
{
  if (!goal_active_.load()) return;

  NavigateSDNL::Goal goal;
  std::shared_ptr<GoalHandleNavigate> gh;
  geometry_msgs::msg::Pose2D pose;
  bool have_pose = false;

  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    gh = active_goal_handle_;
    goal = active_goal_;
  }
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    have_pose = have_pose_;
    if (have_pose) pose = last_pose_;
  }

  if (!gh) { goal_active_.store(false); return; }

  if (!have_pose) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No robot_localisation yet.");
    publishZeroCmd();
    return;
  }

  if (cancel_requested_.load() || gh->is_canceling()) {
    publishZeroCmd();
    auto result = std::make_shared<NavigateSDNL::Result>();
    result->success = false;
    result->message = "Canceled";
    gh->canceled(result);
    goal_active_.store(false);
    cancel_requested_.store(false);
    return;
  }

  const double dx = goal.target_pose.x - pose.x;
  const double dy = goal.target_pose.y - pose.y;
  const double dist = std::hypot(dx, dy);

  const double reached_radius = (goal.reached_radius > 0.0f) ? goal.reached_radius : static_cast<float>(default_reached_radius_);
  const double yaw_tol = (goal.yaw_tolerance > 0.0f) ? goal.yaw_tolerance : static_cast<float>(default_yaw_tolerance_);

  const double yaw_err = sdnl::wrap_pi(static_cast<double>(goal.target_pose.theta) - static_cast<double>(pose.theta));

  const double v_max = (goal.max_linear_speed > 0.0f) ? goal.max_linear_speed : static_cast<float>(default_max_linear_speed_);
  const double w_max = (goal.max_angular_speed > 0.0f) ? goal.max_angular_speed : static_cast<float>(default_max_angular_speed_);

  const double psi_target_map = std::atan2(dy, dx);
  const double psi_target_base = sdnl::wrap_pi(psi_target_map - pose.theta);

  geometry_msgs::msg::Twist cmd;
  if (dist > reached_radius) {
    cmd.linear.x = v_max;
    cmd.angular.z = sdnl::clamp(1.5 * psi_target_base, -w_max, w_max);
  } else {
    cmd.linear.x = 0.0;
    cmd.angular.z = sdnl::clamp(2.0 * yaw_err, -w_max, w_max);
  }

  cmd_vel_pub_->publish(cmd);

  auto fb = std::make_shared<NavigateSDNL::Feedback>();
  fb->dist_to_target = static_cast<float>(dist);
  fb->yaw_error = static_cast<float>(yaw_err);
  fb->cmd_vel = cmd;
  fb->min_obstacle_dist_edge = -1.0f;
  fb->state = (dist > reached_radius) ? "moving" : "rotating";
  gh->publish_feedback(fb);

  {
    std::lock_guard<std::mutex> lk(debug_mutex_);
    latest_cmd_ = cmd;
    latest_dist_ = dist;
    latest_yaw_err_ = yaw_err;
    latest_psi_target_base_ = psi_target_base;
  }

  if (dist <= reached_radius && std::abs(yaw_err) <= yaw_tol) {
    publishZeroCmd();
    auto result = std::make_shared<NavigateSDNL::Result>();
    result->success = true;
    result->message = "Reached target pose";
    gh->succeed(result);
    goal_active_.store(false);
  }
}

void SDNLNavNode::debugLoop()
{
  if (!debug_enabled_ || !debug_pub_) return;
  if (!goal_active_.load()) return;

  NavigateSDNL::Goal goal;
  geometry_msgs::msg::Pose2D pose;
  bool have_pose = false;

  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    goal = active_goal_;
  }
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    have_pose = have_pose_;
    if (have_pose) pose = last_pose_;
  }
  if (!have_pose) return;

  geometry_msgs::msg::Twist cmd;
  double dist = 0.0, yaw_err = 0.0, psi_target_base = 0.0;
  {
    std::lock_guard<std::mutex> lk(debug_mutex_);
    cmd = latest_cmd_;
    dist = latest_dist_;
    yaw_err = latest_yaw_err_;
    psi_target_base = latest_psi_target_base_;
  }

  std::vector<float> y_att, y_rep, y_final;
  debug_builder_.build_curves(n_samples_, psi_target_base, goal.ignore_obstacles, y_att, y_rep, y_final);

  charmie_interfaces::msg::SDNLDebug dbg;
  dbg.header.stamp = this->now();
  dbg.header.frame_id = "base_link";
  dbg.n_samples = static_cast<uint16_t>(y_att.size());
  dbg.dtheta = static_cast<float>((2.0 * M_PI) / static_cast<double>(std::max<size_t>(1, y_att.size())));
  dbg.y_attractor = y_att;
  dbg.y_rep_sum = y_rep;
  dbg.y_final = y_final;

  dbg.robot_map = pose;
  dbg.target_pose = goal.target_pose;
  dbg.psi_target_base = static_cast<float>(psi_target_base);

  dbg.f_target = 0.0f;
  dbg.f_obstacle = 0.0f;
  dbg.f_final = 0.0f;

  dbg.cmd_vel = cmd;
  dbg.dist_to_target = static_cast<float>(dist);
  dbg.min_obstacle_dist_edge = -1.0f;

  debug_pub_->publish(dbg);
}
