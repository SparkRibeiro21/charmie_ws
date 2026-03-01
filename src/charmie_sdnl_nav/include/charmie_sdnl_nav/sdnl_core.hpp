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
  bool use_obstacle_cutoff = true;
  
  double tar_min_dist    = 0.20; // distance where slowdown for target is maximum (0 means only at reached radius)
  double tar_slow_dist   = 1.00; // distance where slowdown for target begins (measured from reached_radius)
  double v_tar_min_frac  = 0.20; // minimum allowed velocity fraction for target slowdown
  double obs_min_dist    = 0.30; // distance where slowdown for obstacles is maximum
  double obs_slow_dist   = 1.00; // distance where slowdown for obstacles begins
  double v_obs_min_frac  = 0.10; // minimum allowed velocity fraction for obstacles 
  double v_turn_min_frac = 0.10; // minimum allowed velocity fraction when turning
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

struct ObstacleTerm
{
  double psi_obs{0.0};       // radians, base frame, 0 forward, +left (CCW)
  double d_obs_edge{0.0};    // meters, distance from robot *edge* (>= 0)
  double delta_theta{0.0};   // radians, angular span of the obstacle term
};

struct RepulsorPrecomp
{
  double psi;            // direction (either base or map, depending on context)
  double lambda;         // beta1 * exp(-d/beta2)
  double sigma;          // computed from delta_theta and d
  double inv_two_sigma2; // avoids recomputation for divisions and exponentials
};

struct SdnlInput
{
  Pose2D robot_map;
  Target2D target_map;

  bool ignore_obstacles{true};  // attractor-only stage uses true by default
  int n_samples{720};

  RadarData radar;              // optional; radar.valid=false means “no radar”

  bool compute_curves{false};   // Only true when debug publishing is enabled

  // Control constraints (provided by node: defaults or action goal)
  double v_max{0.0};          // [m/s] clamp for linear command
  double w_max{0.0};          // [rad/s] clamp for angular command
  double reached_radius{0.0}; // [m] used to define slow_dist to target

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

  // Obstacle info
  double min_obstacle_dist_edge{std::numeric_limits<double>::infinity()}; 
  std::vector<ObstacleTerm> obstacles;

  int n_repulsors{0};                 // number of repulsor curves
  std::vector<float> y_rep_all_flat;  // size = n_repulsors * n_samples

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

  // Final suggested commands (controller output)
  double v_cmd{0.0};  // [m/s]
  double w_cmd{0.0};  // [rad/s]

  // For debug only
  double s_target{1.0};
  double s_obs{1.0};
  double s_turn{1.0};
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
  
  // Main entry: computes SDNL math (attractor + repulsors + optional curves + cmd_vel).
  SdnlOutput compute(const SdnlInput& in) const;

  // Compute attractor curve y_att(phi) over [-pi, +pi]
  void compute_attractor_curve(
    int n_samples,
    double psi_target_map,
    bool ignore_obstacles,
    std::vector<float>& y_att_out) const;

private:

  static void map_radar_to_obstacles(
    const RadarData& radar,
    double robot_radius,
    double max_dist_for_obs,
    bool use_obstacle_cutoff,
    std::vector<ObstacleTerm>& out_terms,
    double& out_min_edge_dist);

  static void build_repulsors_base(
    const std::vector<ObstacleTerm>& obstacles,
    const SdnlParams& p,
    std::vector<RepulsorPrecomp>& rep_out);

  static void build_repulsors_map(
    const std::vector<ObstacleTerm>& obstacles,
    double robot_yaw_map,
    const SdnlParams& p,
    std::vector<RepulsorPrecomp>& rep_out);

  static double eval_repulsor_sum(
    double phi,  // angle in the same domain as repulsors.psi
    const std::vector<RepulsorPrecomp>& reps);

  static void compute_repulsor_curves_map(
    int N,
    const std::vector<RepulsorPrecomp>& reps_map,
    std::vector<float>& y_rep_sum,
    std::vector<float>& y_rep_all_flat); // Flattened row-major: y_rep_all_flat[k*N + i]

  static double compute_v_cmd(
    double dist_to_target,
    double reached_radius,
    double min_obstacle_dist_edge,
    bool ignore_obstacles,
    double v_max,
    double turn_frac,   // 0..1
    const SdnlParams& p,
    double& s_target,
    double& s_obs,
    double& s_turn);

  static double smoothstep01(double x);

  SdnlParams params_;
};

}  // namespace sdnl