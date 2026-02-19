#pragma once
#include <vector>
#include "charmie_sdnl_nav/sdnl_core.hpp"

namespace sdnl
{
class DebugBuilder
{
public:
  explicit DebugBuilder(const SdnlCore& core);

  // Build curves for plotting (y_att, y_rep_sum, y_final)
  void build_curves(
    int n_samples,
    double psi_target_base,
    bool ignore_obstacles,
    std::vector<float>& y_att,
    std::vector<float>& y_rep_sum,
    std::vector<float>& y_final) const;

private:
  const SdnlCore& core_;
};
}  // namespace sdnl
