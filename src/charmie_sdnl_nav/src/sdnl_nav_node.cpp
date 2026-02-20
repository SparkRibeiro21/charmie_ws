#include "charmie_sdnl_nav/sdnl_nav_node.hpp"
#include "charmie_sdnl_nav/angle_utils.hpp"

#include <chrono>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

SDNLNavNode::SDNLNavNode()
: Node("sdnl_nav"),
  debug_test_signal_(this->declare_parameter<bool>("debug_test_signal", true)),
  control_rate_hz_(this->declare_parameter<double>("control_rate_hz", 30.0)),
  debug_enabled_(this->declare_parameter<bool>("debug_enabled", true)),
  debug_rate_hz_(this->declare_parameter<double>("debug_rate_hz", 10.0)),
  n_samples_(this->declare_parameter<int>("n_samples", 720)),
  topic_robot_localisation_(this->declare_parameter<std::string>("topic_robot_localisation", "robot_localisation")),
  topic_radar_data_(this->declare_parameter<std::string>("topic_radar_data", "radar/data")),
  topic_cmd_vel_(this->declare_parameter<std::string>("topic_cmd_vel", "cmd_vel")),
  topic_sdnl_debug_(this->declare_parameter<std::string>("topic_sdnl_debug", "sdnl/debug")),
  topic_sdnl_marker_debug_(this->declare_parameter<std::string>("topic_sdnl_marker_debug", "sdnl/marker_debug")),
  marker_debug_rate_hz_(this->declare_parameter<double>("marker_debug_rate_hz", 5.0)),
  default_max_linear_speed_(this->declare_parameter<double>("default_max_linear_speed", 0.4)),
  default_max_angular_speed_(this->declare_parameter<double>("default_max_angular_speed", 1.0)),
  default_reached_radius_(this->declare_parameter<double>("default_reached_radius", 0.6)),
  default_yaw_tolerance_(this->declare_parameter<double>("default_yaw_tolerance", 0.26)),
  radar_timeout_s_(this->declare_parameter<double>("radar_timeout_s", 1.0)),
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

  marker_debug_pub_ = this->create_publisher<charmie_interfaces::msg::SDNLMarkerDebug>(topic_sdnl_marker_debug_, 10);

  pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
    topic_robot_localisation_, 10,
    std::bind(&SDNLNavNode::poseCallback, this, std::placeholders::_1));

  radar_sub_ = this->create_subscription<charmie_interfaces::msg::RadarData>(
    topic_radar_data_, 10,
    std::bind(&SDNLNavNode::radarCallback, this, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<NavigateSDNL>(
    this,
    "sdnl_navigate_to_pose",
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

  const auto marker_period = std::chrono::duration<double>(1.0 / std::max(1e-6, marker_debug_rate_hz_));
  marker_debug_timer_ = this->create_wall_timer(
  std::chrono::duration_cast<std::chrono::nanoseconds>(marker_period),
  [this]()
  {
    // Make it robust even if RViz starts late: always publish something
    if (!goal_active_.load()) {
      publishMarkerDebug(false, nullptr);
      return;
    }

    NavigateSDNL::Goal g;
    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      g = active_goal_;
    }
    publishMarkerDebug(true, &g);
  });

  RCLCPP_INFO(this->get_logger(), "SDNLNavNode started (control_rate=%.1f Hz, debug=%s).",
              control_rate_hz_, debug_enabled_ ? "true" : "false");
}

rclcpp_action::GoalResponse SDNLNavNode::handle_goal(
  const rclcpp_action::GoalUUID&,
  std::shared_ptr<const NavigateSDNL::Goal> goal)
{
  (void)goal;

  // exemplo: se não tens pose ainda, rejeita
  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    if (!have_pose_) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: no robot_localisation yet.");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SDNLNavNode::handle_cancel(
  const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  std::lock_guard<std::mutex> lk(goal_mutex_);

  // Só aceitamos cancel do goal ativo
  if (active_goal_handle_ && goal_handle == active_goal_handle_) {
    const auto & g = active_goal_;
    RCLCPP_WARN(this->get_logger(),
      "Cancel requested for active goal: x=%.3f y=%.3f theta=%.3f rad",
      g.target_pose.x, g.target_pose.y, g.target_pose.theta);

    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  RCLCPP_WARN(this->get_logger(), "Cancel requested for NON-active goal -> REJECT");
  return rclcpp_action::CancelResponse::REJECT;
}

void SDNLNavNode::handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
{
  std::shared_ptr<GoalHandleNavigate> old_gh;
  NavigateSDNL::Goal old_goal_copy;
  NavigateSDNL::Goal new_goal_copy = *(goal_handle->get_goal());

  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    old_gh = active_goal_handle_;
    old_goal_copy = active_goal_;

    active_goal_handle_ = goal_handle;
    active_goal_ = new_goal_copy;
    goal_active_.store(true);
    cancel_requested_.store(false);
  }
  
  publishMarkerDebug(true, &new_goal_copy);

  if (old_gh && old_gh->is_active()) {
    RCLCPP_WARN(this->get_logger(),
      "Preempting previous goal: old(x=%.3f y=%.3f th=%.3f) -> new(x=%.3f y=%.3f th=%.3f)",
      old_goal_copy.target_pose.x, old_goal_copy.target_pose.y, old_goal_copy.target_pose.theta,
      new_goal_copy.target_pose.x, new_goal_copy.target_pose.y, new_goal_copy.target_pose.theta);

    publishZeroCmd();

    auto result = std::make_shared<NavigateSDNL::Result>();
    result->success = false;
    result->message = "Preempted by a new goal";
    old_gh->abort(result);
  }

  const double theta_deg = new_goal_copy.target_pose.theta * 180.0 / M_PI;
  RCLCPP_INFO(this->get_logger(),
    "Accepted NavigateSDNL goal: x=%.3f y=%.3f theta=%.3f rad (%.1f deg) (mode=%u ignore_obs=%s)",
    new_goal_copy.target_pose.x, new_goal_copy.target_pose.y, new_goal_copy.target_pose.theta, theta_deg,
    new_goal_copy.mode, new_goal_copy.ignore_obstacles ? "true" : "false");
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
  last_radar_stamp_ = rclcpp::Time(msg->header.stamp);
}

void SDNLNavNode::publishZeroCmd()
{
  geometry_msgs::msg::Twist t;
  cmd_vel_pub_->publish(t);
}

void SDNLNavNode::publishMarkerDebug(bool active, const NavigateSDNL::Goal* goal_ptr)
{
  if (!marker_debug_pub_) return;

  charmie_interfaces::msg::SDNLMarkerDebug m;
  m.header.stamp = this->now();
  m.header.frame_id = "map";   // important: RViz cylinder should be in map frame

  m.active = active;

  if (goal_ptr) {
    m.target_pose = goal_ptr->target_pose;
    m.reached_radius = (goal_ptr->reached_radius > 0.0f) ? goal_ptr->reached_radius : static_cast<float>(default_reached_radius_);
    m.mode = goal_ptr->mode;
    m.ignore_obstacles = goal_ptr->ignore_obstacles;
  } else {
    // When inactive, it's still useful to publish something consistent
    m.target_pose.x = 0.0;
    m.target_pose.y = 0.0;
    m.target_pose.theta = 0.0;
    m.reached_radius = 0.0f;
    m.mode = 0;
    m.ignore_obstacles = false;
  }

  marker_debug_pub_->publish(m);

  // Optional cache
  {
    std::lock_guard<std::mutex> lk(marker_debug_mutex_);
    last_marker_debug_ = m;
  }
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

  charmie_interfaces::msg::RadarData radar;
  bool have_radar = false;
  rclcpp::Time radar_stamp(0, 0, this->get_clock()->get_clock_type());

  {
    std::lock_guard<std::mutex> lk(data_mutex_);
    have_radar = have_radar_;
    if (have_radar) {
      radar = last_radar_;
      radar_stamp = last_radar_stamp_;
    }
  }

  const double radar_age_s =
    have_radar ? (this->now() - radar_stamp).seconds()
               : std::numeric_limits<double>::infinity();

  const bool radar_stale = (!have_radar) || (radar_age_s > radar_timeout_s_);

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Radar status: have=%s stale=%s age=%.3fs (timeout=%.3fs) sectors=%u",
    have_radar ? "true" : "false",
    radar_stale ? "true" : "false",
    radar_age_s, radar_timeout_s_,
    have_radar ? radar.number_of_sectors : 0);

  // Compute min obstacle distance to robot edge (debug metric only)
  bool any_point = false;
  double min_dist_center = std::numeric_limits<double>::infinity();

  if (!radar_stale) {
    for (const auto & s : radar.sectors) {
      if (!s.has_point) continue;
      any_point = true;
      min_dist_center = std::min(min_dist_center, static_cast<double>(s.min_distance));
    }
  }

  // Use the existing ROS parameter (no dependency on SdnlCore API)
  const double robot_radius = this->get_parameter("robot_radius").as_double();

  const double min_dist_edge =
    any_point ? (min_dist_center - robot_radius)
              : std::numeric_limits<double>::infinity();

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

    if (gh->is_canceling()) {
      gh->canceled(result);   // ✅ valid
    } else {
      // Not yet CANCELING (race) -> abort is always valid from EXECUTING
      result->message = "Canceled (server-side abort race)";
      gh->abort(result);
    }

    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      active_goal_handle_.reset();
    }
    goal_active_.store(false);
    cancel_requested_.store(false);
    publishMarkerDebug(false, nullptr);
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
  fb->min_obstacle_dist_edge = std::isfinite(min_dist_edge) ? static_cast<float>(min_dist_edge) : -1.0f;
  fb->state = (dist > reached_radius) ? "moving" : "rotating";
  gh->publish_feedback(fb);

  {
    std::lock_guard<std::mutex> lk(debug_mutex_);
    latest_cmd_ = cmd;
    latest_dist_ = dist;
    latest_yaw_err_ = yaw_err;
    latest_psi_target_base_ = psi_target_base;
    latest_min_obs_edge_ = min_dist_edge;
  }

  if (dist <= reached_radius && std::abs(yaw_err) <= yaw_tol) {
    const auto & g = goal;

    RCLCPP_INFO(this->get_logger(),
      "SDNL goal SUCCEEDED: x=%.3f y=%.3f theta=%.3f rad | final dist=%.3f yaw_err=%.3f",
      g.target_pose.x, g.target_pose.y, g.target_pose.theta,
      dist, yaw_err);

    publishZeroCmd();

    auto result = std::make_shared<NavigateSDNL::Result>();
    result->success = true;
    result->message = "Reached target pose";
    gh->succeed(result);

    {
      std::lock_guard<std::mutex> lk(goal_mutex_);
      active_goal_handle_.reset();
    }
    goal_active_.store(false);

    // publish marker debug immediately (inactive)
    publishMarkerDebug(false, nullptr);

    return;
  }
}

void SDNLNavNode::debugLoop()
{
  if (!debug_enabled_ || !debug_pub_) return;

  const int N = std::max(64, n_samples_);
  const double dtheta = (2.0 * M_PI) / static_cast<double>(N);
  const double t = this->now().seconds();

  std::vector<float> y_att(N), y_rep(N), y_final(N);

  for (int i = 0; i < N; ++i) {
    const double th = i * dtheta;
    y_att[i] = static_cast<float>(10.0 * std::sin(th - 0.5 * t));
    y_rep[i] = static_cast<float>( 6.0 * std::cos(2.0 * th + 0.3 * t));
    y_final[i] = y_att[i] + y_rep[i];
  }

  charmie_interfaces::msg::SDNLDebug dbg;
  dbg.header.stamp = this->now();
  dbg.header.frame_id = "base_link";
  dbg.n_samples = static_cast<uint16_t>(N);
  dbg.dtheta = static_cast<float>(dtheta);

  dbg.y_attractor = y_att;
  dbg.y_rep_sum   = y_rep;
  dbg.y_final     = y_final;

  // Minimal “fake” metadata so viewer can draw heading line etc.
  dbg.robot_map.x = 0.0;
  dbg.robot_map.y = 0.0;
  dbg.robot_map.theta = 0.0;

  dbg.target_pose.x = 1.0;
  dbg.target_pose.y = 0.0;
  dbg.target_pose.theta = 0.0;

  // Animate psi so you can see the red line move
  dbg.psi_target_base = static_cast<float>(std::fmod(std::max(0.0, t * 0.3), 2.0 * M_PI));

  dbg.f_target = 0.0f;
  dbg.f_obstacle = 0.0f;
  dbg.f_final = 0.0f;

  dbg.cmd_vel = geometry_msgs::msg::Twist(); // zeros
  dbg.dist_to_target = 0.0f;
  dbg.min_obstacle_dist_edge = -1.0f;

  debug_pub_->publish(dbg);
}
