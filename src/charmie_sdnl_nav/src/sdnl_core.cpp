#include "charmie_sdnl_nav/sdnl_core.hpp"
#include <cmath>

namespace sdnl
{
SdnlCore::SdnlCore(SdnlParams p) : params_(p) {}

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
    const double th = static_cast<double>(i) * dtheta;
    const double y = -lambda * std::sin(th - psi_target_base);
    y_att_out[static_cast<size_t>(i)] = static_cast<float>(y);
  }
}
}  // namespace sdnl
