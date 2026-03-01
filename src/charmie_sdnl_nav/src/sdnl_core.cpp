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

  // -------------------------
  // Scalars (ALWAYS base-domain)
  // -------------------------
  const double lambda_t = in.ignore_obstacles ? params_.lambda_target_not_obs
                                             : params_.lambda_target_with_obs;

  // Evaluate attractor scalar at phi_b = 0 (front of robot), base-domain:
  // f_target = -lambda*sin(0 - psi_target_base) = lambda*sin(psi_target_base)
  out.f_target = lambda_t * std::sin(out.psi_target_base);

  out.f_obstacle = 0.0;
  if (!in.ignore_obstacles && !out.obstacles.empty()) {
    std::vector<RepulsorPrecomp> reps_base;
    build_repulsors_base(out.obstacles, params_, reps_base);
    // Evaluate at phi_b = 0
    out.f_obstacle = eval_repulsor_sum(0.0, reps_base);
  }

  out.f_final = out.f_target + out.f_obstacle;

  // -------------------------
  // Commands (base-domain)
  // -------------------------
  // Angular from SDNL final scalar (clamped)
  out.w_cmd = sdnl::clamp(out.f_final, -in.w_max, in.w_max);

  const double turn_frac = (in.w_max > 1e-6) ? std::min(1.0, std::abs(out.w_cmd) / in.w_max) : 0.0;

  out.v_cmd = compute_v_cmd(
    out.dist_to_target,
    in.reached_radius,
    out.min_obstacle_dist_edge,
    in.ignore_obstacles,
    in.v_max,
    turn_frac,        // 0..1
    params_,
    out.s_target,
    out.s_obs,
    out.s_turn
  );

  // -------------------------
  // Curves (ALWAYS map-domain, only if compute_curves)
  // -------------------------
  out.y_attractor.clear();
  out.y_rep_sum.clear();
  out.y_final.clear();
  out.n_repulsors = 0;
  out.y_rep_all_flat.clear();

  if (in.compute_curves) {

    // Map-domain debug target bearing:
    // psi_target_map_debug = psi_target_base + robot_yaw_map
    const double psi_target_map_debug = sdnl::wrap_pi(out.psi_target_base + in.robot_map.theta);

    // Attractor curve in MAP domain (stable in viewer)
    compute_attractor_curve(N, psi_target_map_debug, in.ignore_obstacles, out.y_attractor);

    // Repulsor curves in MAP domain (stable in viewer)
    if (!in.ignore_obstacles && !out.obstacles.empty()) {

      std::vector<RepulsorPrecomp> reps_map;
      build_repulsors_map(out.obstacles, in.robot_map.theta, params_, reps_map);

      out.n_repulsors = static_cast<int>(reps_map.size());
      compute_repulsor_curves_map(N, reps_map, out.y_rep_sum, out.y_rep_all_flat);

    } else {
      out.y_rep_sum.assign(out.y_attractor.size(), 0.0f);
      out.n_repulsors = 0;
      out.y_rep_all_flat.clear();
    }

    // Final curve
    out.y_final.resize(out.y_attractor.size(), 0.0f);
    for (size_t i = 0; i < out.y_final.size(); ++i) {
      out.y_final[i] = out.y_attractor[i] + out.y_rep_sum[i];
    }
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

// -------------------------
// Repulsor helpers (doc eqs. 2-5)
// -------------------------
void SdnlCore::build_repulsors_base(
  const std::vector<ObstacleTerm>& obstacles,
  const SdnlParams& p,
  std::vector<RepulsorPrecomp>& rep_out)
{
  rep_out.clear();
  rep_out.reserve(obstacles.size());

  const double R = p.robot_radius;
  const double beta1 = p.beta1;
  const double beta2 = std::max(1e-9, p.beta2); // avoid div by zero

  for (const auto& o : obstacles) {

    // Distance in the doc/Python is "measured distance" ~ center distance
    const double d_center = std::max(0.0, o.d_obs_edge + R);

    // lambda_i = beta1 * exp(-d/beta2)
    const double lambda = beta1 * std::exp(-d_center / beta2);

    // sigma_i = atan( tan(delta_theta/2) + R/(R + d) )
    const double half_span = 0.5 * std::max(0.0, o.delta_theta);
    const double sigma_raw = std::atan(std::tan(half_span) + (R / (R + d_center)));

    // Numerical guard
    const double sigma = std::max(1e-6, sigma_raw);
    const double inv_two_sigma2 = 1.0 / (2.0 * sigma * sigma);

    RepulsorPrecomp r;
    r.psi = wrap_pi(o.psi_obs);   // base domain
    r.lambda = lambda;
    r.sigma = sigma;
    r.inv_two_sigma2 = inv_two_sigma2;
    rep_out.push_back(r);
  }
}

void SdnlCore::build_repulsors_map(
  const std::vector<ObstacleTerm>& obstacles,
  double robot_yaw_map,
  const SdnlParams& p,
  std::vector<RepulsorPrecomp>& rep_out)
{
  rep_out.clear();
  rep_out.reserve(obstacles.size());

  const double R = p.robot_radius;
  const double beta1 = p.beta1;
  const double beta2 = std::max(1e-9, p.beta2); // avoid div by zero

  for (const auto& o : obstacles) {

    const double d_center = std::max(0.0, o.d_obs_edge + R);
    const double lambda = beta1 * std::exp(-d_center / beta2);

    const double half_span = 0.5 * std::max(0.0, o.delta_theta);
    const double sigma_raw = std::atan(std::tan(half_span) + (R / (R + d_center)));
    const double sigma = std::max(1e-6, sigma_raw);
    const double inv_two_sigma2 = 1.0 / (2.0 * sigma * sigma);

    RepulsorPrecomp r;
    // base -> map for debug curves
    r.psi = wrap_pi(o.psi_obs + robot_yaw_map);
    r.lambda = lambda;
    r.sigma = sigma;
    r.inv_two_sigma2 = inv_two_sigma2;
    rep_out.push_back(r);
  }
}

double SdnlCore::eval_repulsor_sum(
  double phi,
  const std::vector<RepulsorPrecomp>& reps)
{
  double sum = 0.0;

  for (const auto& r : reps) {
    const double d = wrap_pi(phi - r.psi);
    // f_obs,i(phi) = lambda * d * exp(-(d^2)/(2*sigma^2))
    sum += r.lambda * d * std::exp(-(d * d) * r.inv_two_sigma2);
  }

  return sum;
}

void SdnlCore::compute_repulsor_curves_map(
  int N,
  const std::vector<RepulsorPrecomp>& reps_map,
  std::vector<float>& y_rep_sum,
  std::vector<float>& y_rep_all_flat)
{
  if (N < 8) N = 8;

  const int K = static_cast<int>(reps_map.size());
  const double dphi = (2.0 * M_PI) / static_cast<double>(N);

  y_rep_sum.assign(static_cast<size_t>(N), 0.0f);
  y_rep_all_flat.assign(static_cast<size_t>(K) * static_cast<size_t>(N), 0.0f);

  for (int i = 0; i < N; ++i) {
    const double phi = -M_PI + static_cast<double>(i) * dphi;

    double s = 0.0;
    for (int k = 0; k < K; ++k) {
      const auto& r = reps_map[static_cast<size_t>(k)];
      const double d = wrap_pi(phi - r.psi);
      const double v = r.lambda * d * std::exp(-(d * d) * r.inv_two_sigma2);

      y_rep_all_flat[static_cast<size_t>(k) * static_cast<size_t>(N) + static_cast<size_t>(i)] =
        static_cast<float>(v);

      s += v;
    }

    y_rep_sum[static_cast<size_t>(i)] = static_cast<float>(s);
  }
}


double SdnlCore::smoothstep01(double x)
{
  if (x <= 0.0) return 0.0;
  if (x >= 1.0) return 1.0;
  return x * x * (3.0 - 2.0 * x);  // smoothstep
}

double SdnlCore::compute_v_cmd(
  double dist_to_target,
  double reached_radius,
  double min_obstacle_dist_edge,
  bool ignore_obstacles,
  double v_max,
  double turn_frac,   // 0..1
  const SdnlParams& p,
  double& s_target,
  double& s_obs,
  double& s_turn)
{
  // Defaults (useful when we don't compute a factor)
  s_target = 1.0;
  s_obs    = 1.0;
  s_turn   = 1.0;
  
  // -------------------------
  // 1) Target slowdown
  // -------------------------
  const double rr = std::max(1e-6, reached_radius);

  // Distances where the target slowdown starts / reaches minimum (offset by reached radius)
  const double d_min  = std::max(0.0, rr + p.tar_min_dist);
  const double d_slow = std::max(d_min + 1e-6, rr + p.tar_slow_dist);

  double target_factor = 1.0;

  if (dist_to_target <= d_min) {
    // closest zone -> enforce minimum speed fraction
    target_factor = std::clamp(p.v_tar_min_frac, 0.0, 1.0);
  }
  else if (dist_to_target < d_slow) {
    // Map dist: [d_min .. d_slow] -> x: [0 .. 1]
    const double x = (dist_to_target - d_min) / (d_slow - d_min);  // 0..1
    const double s = smoothstep01(x);                              // 0..1

    const double vmin = std::clamp(p.v_tar_min_frac, 0.0, 1.0);
    target_factor = vmin + (1.0 - vmin) * s;  // s=0 -> vmin, s=1 -> 1.0
  }
  else {
    target_factor = 1.0;
  }

  // -------------------------
  // 2) Obstacles slowdown
  // -------------------------
  double obs_factor = 1.0;

  if (ignore_obstacles) {
    obs_factor = 1.0;
  }
  else if (std::isfinite(min_obstacle_dist_edge)) {

    const double d_min  = std::max(0.0, p.obs_min_dist);
    const double d_slow = std::max(d_min + 1e-6, p.obs_slow_dist);

    // Map distance to x in [0..1]
    // d <= d_min  -> x = 0   (max slowdown)
    // d >= d_slow -> x = 1   (no slowdown)
    double x = 1.0;

    if (min_obstacle_dist_edge <= d_min) {
      x = 0.0;
    }
    else if (min_obstacle_dist_edge < d_slow) {
      x = (min_obstacle_dist_edge - d_min) / (d_slow - d_min);
    }
    else {
      x = 1.0;
    }

    const double s = smoothstep01(x);  // smooth 0..1

    const double vmin = std::clamp(p.v_obs_min_frac, 0.0, 1.0);

    // s=0 -> vmin
    // s=1 -> 1.0
    obs_factor = vmin + (1.0 - vmin) * s;
  }
  else {
    obs_factor = 1.0;
  }

  // -------------------------
  // 3) Turn slowdown
  // -------------------------
  const double a = std::clamp(turn_frac, 0.0, 1.0);          // 0..1
  const double vmin = std::clamp(p.v_turn_min_frac, 0.0, 1.0);

  // Queremos um "x" que seja 1 quando a=0 e 0 quando a=1,
  // para smoothstep nos dar 1 -> 0 suavemente.
  const double x = 1.0 - a;                                  // 1..0

  // smoothstep01(x): 1 (a=0) -> 0 (a=1)
  const double s = smoothstep01(x);

  // mapeia: s=1 -> 1.0 ; s=0 -> vmin
  const double turn_factor = vmin + (1.0 - vmin) * s;

  // Export debug scalars
  s_target = target_factor;
  s_obs    = obs_factor;
  s_turn   = turn_factor;

  // Use the most restrictive factor
  const double factor = std::min({s_target, s_obs, s_turn});
  return std::max(0.0, v_max * factor);
}

}  // namespace sdnl