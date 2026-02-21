#pragma once

#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>

namespace sdnl
{

// -------------------------
// Parameters (tunable)
// -------------------------
struct SdnlParams
{
  double lambda_target_not_obs = 10.0;
  double lambda_target_with_obs = 4.0;
  double beta1 = 40.0;
  double beta2 = 0.06;
  double max_dist_for_obs = 2.0;
  double robot_radius = 0.28;
  double heading_offset_rad = 0.0;  // optional offset applied in target->base angle computation
};

// -------------------------
// Pure-math types (ROS-free)
// -------------------------
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};  // radians, map frame
};

struct Target2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};  // radians, map frame (desired final yaw)
};

struct RadarSector
{
  double start_angle{0.0};   // radians (base frame convention from your radar node)
  double end_angle{0.0};     // radians
  double min_distance{0.0};  // meters (distance from robot center)
  bool has_point{false};
};

struct RadarData
{
  bool valid{false};
  std::vector<RadarSector> sectors;
};

struct SdnlInput
{
  Pose2D robot_map;
  Target2D target_map;

  bool ignore_obstacles{true};  // attractor-only stage uses true by default
  int n_samples{720};

  RadarData radar;              // optional; radar.valid=false means “no radar”

  bool compute_curves{false};   // Only true when debug publishing is enabled
};

struct SdnlOutput
{
  // Geometry / angles
  double dx{0.0};
  double dy{0.0};
  double dist_to_target{0.0};

  double psi_target_map{0.0};   // atan2(dy, dx)
  double psi_target_base{0.0};  // wrap_pi(psi_target_map - robot.theta + heading_offset)
  double yaw_error{0.0};        // wrap_pi(target.theta - robot.theta)

  // Obstacle debug metric (optional)
  double min_obstacle_dist_edge{std::numeric_limits<double>::infinity()}; // (min center dist - robot_radius)

  // Curves
  int n_samples{0};
  double dtheta{0.0};
  std::vector<float> y_attractor;
  std::vector<float> y_rep_sum;
  std::vector<float> y_final;

  // SDNL scalar terms (useful for debug & later repulsion integration)
  double f_target{0.0};
  double f_obstacle{0.0};
  double f_final{0.0};
};

// -------------------------
// Core
// -------------------------
class SdnlCore
{
public:
  explicit SdnlCore(SdnlParams p);

  const SdnlParams& params() const { return params_; }
  void set_params(const SdnlParams& p) { params_ = p; }

  // One main entry point: computes all SDNL math (attractor-only for now).
  SdnlOutput compute(const SdnlInput& in) const;

  // Compute attractor curve y_att(theta) over [0, 2pi)
  void compute_attractor_curve(
    int n_samples,
    double psi_target_base,
    bool ignore_obstacles,
    std::vector<float>& y_att_out) const;

private:
  static double wrap_pi(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));  // [-pi, pi]
  }

  static double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  SdnlParams params_;
};

}  // namespace sdnl