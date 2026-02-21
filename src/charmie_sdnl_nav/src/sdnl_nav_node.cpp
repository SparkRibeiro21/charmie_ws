#include "charmie_sdnl_nav/sdnl_nav_node.hpp"
#include "charmie_sdnl_nav/angle_utils.hpp"

#include <chrono>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

SDNLNavNode::SDNLNavNode()
: Node("sdnl_nav"),
  control_rate_hz_(this->declare_parameter<double>("control_rate_hz", 30.0)),
  debug_enabled_(this->declare_parameter<bool>("debug_enabled", true)),
  debug_rate_hz_(this->declare_parameter<double>("debug_rate_hz", 10.0)),
  n_samples_(this->declare_parameter<int>("n_samples", 720)),
  marker_debug_rate_hz_(this->declare_parameter<double>("marker_debug_rate_hz", 5.0)),
  radar_timeout_s_(this->declare_parameter<double>("radar_timeout_s", 1.0)),
  topic_robot_localisation_(this->declare_parameter<std::string>("topic_robot_localisation", "robot_localisation")),
  topic_radar_data_(this->declare_parameter<std::string>("topic_radar_data", "radar/data")),
  topic_cmd_vel_(this->declare_parameter<std::string>("topic_cmd_vel", "cmd_vel")),
  topic_sdnl_debug_(this->declare_parameter<std::string>("topic_sdnl_debug", "sdnl/debug")),
  topic_sdnl_marker_debug_(this->declare_parameter<std::string>("topic_sdnl_marker_debug", "sdnl/marker_debug")),
  default_max_linear_speed_(this->declare_parameter<double>("default_max_linear_speed", 0.42)),
  default_max_angular_speed_(this->declare_parameter<double>("default_max_angular_speed", 0.9)),
  default_reached_radius_(this->declare_parameter<double>("default_reached_radius", 0.5)),
  default_yaw_tolerance_(this->declare_parameter<double>("default_yaw_tolerance", 0.052359908)), // 3 degrees in radians

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
  }())

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
    
    {
    std::lock_guard<std::mutex> lk(debug_mutex_);
    latest_have_pose_for_debug_ = false;
    latest_have_radar_for_debug_ = false;
    }
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

  if (!goal_active_.load()) {
    {
      std::lock_guard<std::mutex> lk(debug_mutex_);
      latest_have_pose_for_debug_ = false;
      latest_have_radar_for_debug_ = false;
    }
    return;
  }

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

  if (!gh) {
    goal_active_.store(false);
    {
      std::lock_guard<std::mutex> lk(debug_mutex_);
      latest_have_pose_for_debug_ = false;
      latest_have_radar_for_debug_ = false;
    }
    return;
  }

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

    {
    std::lock_guard<std::mutex> lk(debug_mutex_);
    latest_have_pose_for_debug_ = false;
    latest_have_radar_for_debug_ = false;
    }

    cancel_requested_.store(false);
    publishMarkerDebug(false, nullptr);
    return;
  }

  sdnl::SdnlInput in;
  in.robot_map = {pose.x, pose.y, pose.theta};
  in.target_map = {goal.target_pose.x, goal.target_pose.y, goal.target_pose.theta};
  in.ignore_obstacles = goal.ignore_obstacles;
  in.n_samples = n_samples_;      // not used when compute_curves=false, but fine
  in.compute_curves = false;      // ✅ always false in controlLoop

  // Radar: pass only if fresh (optional, but good)
  in.radar.valid = !radar_stale;
  if (in.radar.valid) {
    in.radar.sectors.reserve(radar.sectors.size());
    for (const auto& s : radar.sectors) {
      sdnl::RadarSector rs;
      rs.start_angle  = s.start_angle;
      rs.end_angle    = s.end_angle;
      rs.min_distance = s.min_distance;
      rs.has_point    = s.has_point;
      in.radar.sectors.push_back(rs);
    }
  }

  const sdnl::SdnlOutput out = core_.compute(in);
  
  const double reached_radius = (goal.reached_radius > 0.0f) ? goal.reached_radius : static_cast<float>(default_reached_radius_);
  const double yaw_tol = (goal.yaw_tolerance > 0.0f) ? goal.yaw_tolerance : static_cast<float>(default_yaw_tolerance_);
  const double v_max = (goal.max_linear_speed > 0.0f) ? goal.max_linear_speed : static_cast<float>(default_max_linear_speed_);
  const double w_max = (goal.max_angular_speed > 0.0f) ? goal.max_angular_speed : static_cast<float>(default_max_angular_speed_);;

  geometry_msgs::msg::Twist cmd;
  if (out.dist_to_target > reached_radius) {
    cmd.linear.x = v_max;
    cmd.angular.z = sdnl::clamp(1.5 * out.psi_target_base, -w_max, w_max);
  } else {
    cmd.linear.x = 0.0;
    cmd.angular.z = sdnl::clamp(2.0 * out.yaw_error, -w_max, w_max);
  }

  cmd_vel_pub_->publish(cmd);

  auto fb = std::make_shared<NavigateSDNL::Feedback>();
  fb->dist_to_target = static_cast<float>(out.dist_to_target);
  fb->yaw_error = static_cast<float>(out.yaw_error);
  fb->cmd_vel = cmd;
  fb->min_obstacle_dist_edge =
    std::isfinite(out.min_obstacle_dist_edge) ? static_cast<float>(out.min_obstacle_dist_edge) : -1.0f;
  fb->state = (out.dist_to_target > reached_radius) ? "moving" : "rotating";
  gh->publish_feedback(fb);

  {
    std::lock_guard<std::mutex> lk(debug_mutex_);
    latest_cmd_ = cmd;

    latest_pose_for_debug_ = pose;
    latest_goal_for_debug_ = goal;
    latest_have_pose_for_debug_ = have_pose;

    latest_radar_for_debug_ = radar;
    latest_have_radar_for_debug_ = have_radar;
    latest_radar_stamp_for_debug_ = radar_stamp;
  }

  if (out.dist_to_target <= reached_radius && std::abs(out.yaw_error) <= yaw_tol) {
    const auto & g = goal;

    RCLCPP_INFO(this->get_logger(),
      "SDNL goal SUCCEEDED: x=%.3f y=%.3f theta=%.3f rad | final dist=%.3f yaw_err=%.3f",
      g.target_pose.x, g.target_pose.y, g.target_pose.theta,
      out.dist_to_target, out.yaw_error);

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

    {
    std::lock_guard<std::mutex> lk(debug_mutex_);
    latest_have_pose_for_debug_ = false;
    latest_have_radar_for_debug_ = false;
    }

    // publish marker debug immediately (inactive)
    publishMarkerDebug(false, nullptr);

    return;
  }
}

void SDNLNavNode::debugLoop()
{
  if (!debug_enabled_ || !debug_pub_) return;

  // ----------------------------
  // Grab latest snapshot safely
  // ----------------------------
  geometry_msgs::msg::Pose2D pose;
  NavigateSDNL::Goal goal;
  charmie_interfaces::msg::RadarData radar;
  rclcpp::Time radar_stamp(0, 0, this->get_clock()->get_clock_type());
  bool have_pose = false;
  bool have_radar = false;
  geometry_msgs::msg::Twist last_cmd;

  {
    std::lock_guard<std::mutex> lk(debug_mutex_);
    have_pose = latest_have_pose_for_debug_;
    if (have_pose) {
      pose = latest_pose_for_debug_;
      goal = latest_goal_for_debug_;
    }

    have_radar = latest_have_radar_for_debug_;
    if (have_radar) {
      radar = latest_radar_for_debug_;
      radar_stamp = latest_radar_stamp_for_debug_;
    }

    last_cmd = latest_cmd_;
  }

  if (!have_pose) {
    // No pose yet -> nothing meaningful to debug
    return;
  }

  // ----------------------------
  // Radar freshness (same logic)
  // ----------------------------
  const double radar_age_s =
    have_radar ? (this->now() - radar_stamp).seconds()
               : std::numeric_limits<double>::infinity();

  const bool radar_stale = (!have_radar) || (radar_age_s > radar_timeout_s_);

  // ----------------------------
  // Build core input (debug)
  // ----------------------------
  sdnl::SdnlInput in;
  in.robot_map = {pose.x, pose.y, pose.theta};
  in.target_map = {goal.target_pose.x, goal.target_pose.y, goal.target_pose.theta};
  in.ignore_obstacles = goal.ignore_obstacles;

  // Debug wants curves
  in.n_samples = std::max(64, n_samples_);
  in.compute_curves = true;

  // Optional radar
  in.radar.valid = !radar_stale;
  if (in.radar.valid) {
    in.radar.sectors.reserve(radar.sectors.size());
    for (const auto& s : radar.sectors) {
      sdnl::RadarSector rs;
      rs.start_angle  = s.start_angle;
      rs.end_angle    = s.end_angle;
      rs.min_distance = s.min_distance;
      rs.has_point    = s.has_point;
      in.radar.sectors.push_back(rs);
    }
  }

  // Compute full debug data (curves)
  const sdnl::SdnlOutput out = core_.compute(in);

  // Core should have curves now
  if (out.y_attractor.empty()) return;

  // ----------------------------
  // Publish SDNLDebug (REAL)
  // ----------------------------
  charmie_interfaces::msg::SDNLDebug dbg;
  dbg.header.stamp = this->now();
  dbg.header.frame_id = "base_link";  // your viewer expects base_link

  dbg.n_samples = static_cast<uint16_t>(out.n_samples);
  dbg.dtheta = static_cast<float>(out.dtheta);

  dbg.y_attractor = out.y_attractor;
  dbg.y_rep_sum   = out.y_rep_sum;   // zeros for now
  dbg.y_final     = out.y_final;     // equals attractor for now

  // Fill metadata
  dbg.robot_map = pose;
  dbg.target_pose = goal.target_pose;

  dbg.psi_target_base = static_cast<float>(out.psi_target_base);

  dbg.f_target = static_cast<float>(out.f_target);
  dbg.f_obstacle = static_cast<float>(out.f_obstacle);
  dbg.f_final = static_cast<float>(out.f_final);

  dbg.cmd_vel = last_cmd;
  dbg.dist_to_target = static_cast<float>(out.dist_to_target);

  dbg.min_obstacle_dist_edge =
    std::isfinite(out.min_obstacle_dist_edge) ? static_cast<float>(out.min_obstacle_dist_edge) : -1.0f;

  debug_pub_->publish(dbg);
}