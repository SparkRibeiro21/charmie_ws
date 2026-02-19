#pragma once
#include <vector>

namespace sdnl
{
struct SdnlParams
{
  double lambda_target_not_obs = 10.0;
  double lambda_target_with_obs = 4.0;
  double beta1 = 40.0;
  double beta2 = 0.06;
  double max_dist_for_obs = 2.0;
  double robot_radius = 0.28;
  double heading_offset_rad = 0.0;
};

class SdnlCore
{
public:
  explicit SdnlCore(SdnlParams p);

  const SdnlParams& params() const { return params_; }
  void set_params(const SdnlParams& p) { params_ = p; }

  // Compute attractor curve y_att(theta) over [0, 2pi)
  void compute_attractor_curve(
    int n_samples,
    double psi_target_base,
    bool ignore_obstacles,
    std::vector<float>& y_att_out) const;

private:
  SdnlParams params_;
};
}  // namespace sdnl
