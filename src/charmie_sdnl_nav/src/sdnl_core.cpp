#include "charmie_sdnl_nav/sdnl_core.hpp"

namespace sdnl
{

SdnlCore::SdnlCore(SdnlParams p) : params_(p) {}

SdnlOutput SdnlCore::compute(const SdnlInput& in) const
{
  SdnlOutput out;

  // Sanitize samples
  const int N = std::max(8, in.n_samples);
  out.n_samples = N;
  out.dtheta = (2.0 * M_PI) / static_cast<double>(N);

  // Geometry
  out.dx = in.target_map.x - in.robot_map.x;
  out.dy = in.target_map.y - in.robot_map.y;
  out.dist_to_target = std::hypot(out.dx, out.dy);

  // Angles
  out.psi_target_map = std::atan2(out.dy, out.dx);
  out.psi_target_base = wrap_pi(out.psi_target_map - in.robot_map.theta + params_.heading_offset_rad);
  out.yaw_error = wrap_pi(in.target_map.theta - in.robot_map.theta);

  // Obstacle debug metric (cheap; does not change behavior)
  // Compute whenever radar is valid (independent of ignore_obstacles).
  if (in.radar.valid) {
    bool any_point = false;
    double min_dist_center = std::numeric_limits<double>::infinity();

    for (const auto& s : in.radar.sectors) {
      if (!s.has_point) continue;
      any_point = true;
      min_dist_center = std::min(min_dist_center, s.min_distance);
    }

    out.min_obstacle_dist_edge =
      any_point ? (min_dist_center - params_.robot_radius)
                : std::numeric_limits<double>::infinity();
  } else {
    out.min_obstacle_dist_edge = std::numeric_limits<double>::infinity();
  }

  // Default: no curves computed unless requested
  out.y_attractor.clear();
  out.y_rep_sum.clear();
  out.y_final.clear();
  out.f_target = 0.0;
  out.f_obstacle = 0.0;
  out.f_final = 0.0;

  if (in.compute_curves) {
    // Attractor curve
    compute_attractor_curve(N, out.psi_target_base, in.ignore_obstacles, out.y_attractor);

    // Repulsion (later) - for now zeros
    out.y_rep_sum.assign(out.y_attractor.size(), 0.0f);

    // Final curve (later sum) - for now attractor
    out.y_final = out.y_attractor;

    // Debug scalar terms (optional, but helpful for your SDNLDebug)
    const double lambda = in.ignore_obstacles ? params_.lambda_target_not_obs : params_.lambda_target_with_obs;

    // NOTE: this is "instantaneous" attractor value at theta = 0 (convention).
    out.f_target = lambda * std::sin(out.psi_target_base);
    out.f_obstacle = 0.0;
    out.f_final = out.f_target;
  }

  return out;
}

void SdnlCore::compute_attractor_curve(
  int n_samples,
  double psi_target_base,
  bool ignore_obstacles,
  std::vector<float>& y_att_out) const
{
  if (n_samples < 8) n_samples = 8;
  y_att_out.assign(static_cast<size_t>(n_samples), 0.0f);

  const double dtheta = (2.0 * M_PI) / static_cast<double>(n_samples);
  const double lambda = ignore_obstacles ? params_.lambda_target_not_obs : params_.lambda_target_with_obs;

  for (int i = 0; i < n_samples; ++i) {
    // New convention: sample angles cover [-pi, +pi) for red line in sdnl debug to be mostly in and not in side limits of the graphic
    // i=0    -> th = -pi
    // i=N/2  -> th = 0
    // i=N-1  -> th approx +pi - dtheta
    const double th = -M_PI + static_cast<double>(i) * dtheta;

    const double y = -lambda * std::sin(th - psi_target_base);
    y_att_out[static_cast<size_t>(i)] = static_cast<float>(y);
  }
}

}  // namespace sdnl