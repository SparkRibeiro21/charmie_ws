#include "charmie_sdnl_nav/sdnl_core.hpp"
#include "charmie_sdnl_nav/angle_utils.hpp"

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
  out.psi_target_base = sdnl::wrap_pi(out.psi_target_map - in.robot_map.theta + params_.heading_offset_rad);
  out.yaw_error = sdnl::wrap_pi(in.target_map.theta - in.robot_map.theta);

  // Obstacles
  out.obstacles.clear();
  out.min_obstacle_dist_edge = std::numeric_limits<double>::infinity();

  if (in.radar.valid) {
    map_radar_to_obstacles(
      in.radar,
      params_.robot_radius,
      params_.max_dist_for_obs,
      params_.use_obstacle_cutoff,
      out.obstacles,
      out.min_obstacle_dist_edge);
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
    compute_attractor_curve(N, out.psi_target_map, in.ignore_obstacles, out.y_attractor);

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

void SdnlCore::map_radar_to_obstacles(
  const RadarData& radar,
  double robot_radius,
  double max_dist_for_obs,
  bool use_obstacle_cutoff,
  std::vector<ObstacleTerm>& out_terms,
  double& out_min_edge_dist)
{
  out_terms.clear();
  out_min_edge_dist = std::numeric_limits<double>::infinity();

  if (!radar.valid) {
    return;
  }

  // For each sector with a point, create one obstacle term.
  // Assumptions (matches your radar node):
  // - angles are in base frame, 0 forward (+x), +CCW to the left (+y)
  // - min_distance is from robot center (base frame origin)
  for (const auto& s : radar.sectors) {

    if (!s.has_point) {
      continue;
    }

    if (!std::isfinite(s.min_distance)) {
      continue;
    }

    // Convert center-distance -> edge-distance (>= 0)
    const double d_edge = std::max(0.0, s.min_distance - robot_radius);

    // Only keep obstacles within the SDNL obstacle influence range
    if (use_obstacle_cutoff && d_edge > max_dist_for_obs) {
      continue;
    }

    // Sector angular span (should be positive)
    double dtheta = s.end_angle - s.start_angle;

    // If something weird happens (wraps), make it positive in a safe way
    // (Normally your radar builder produces monotonic start/end, so this is just a guard.)
    if (dtheta < 0.0) {
      dtheta = -dtheta;
    }

    // Obstacle "direction": center of the sector
    const double psi_center = 0.5 * (s.start_angle + s.end_angle);

    ObstacleTerm t;
    t.psi_obs = wrap_pi(psi_center);  // use sdnl::wrap_pi from angle_utils.hpp
    t.d_obs_edge = d_edge;
    t.delta_theta = dtheta;

    out_terms.push_back(t);
    out_min_edge_dist = std::min(out_min_edge_dist, d_edge);
  }

  // If no valid obstacles were added, keep infinity
  if (out_terms.empty()) {
    out_min_edge_dist = std::numeric_limits<double>::infinity();
  }
}

void SdnlCore::compute_attractor_curve(
  int n_samples,
  double psi_target_map,
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

    const double y = -lambda * std::sin(th - psi_target_map);
    y_att_out[static_cast<size_t>(i)] = static_cast<float>(y);
  }
}

}  // namespace sdnl