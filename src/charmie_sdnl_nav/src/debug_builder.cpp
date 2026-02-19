#include "charmie_sdnl_nav/debug_builder.hpp"

namespace sdnl
{
DebugBuilder::DebugBuilder(const SdnlCore& core) : core_(core) {}

void DebugBuilder::build_curves(
  int n_samples,
  double psi_target_base,
  bool ignore_obstacles,
  std::vector<float>& y_att,
  std::vector<float>& y_rep_sum,
  std::vector<float>& y_final) const
{
  // For now: repulsor sum = 0. Final = attractor.
  core_.compute_attractor_curve(n_samples, psi_target_base, ignore_obstacles, y_att);

  y_rep_sum.assign(y_att.size(), 0.0f);
  y_final = y_att;
}
}  // namespace sdnl
