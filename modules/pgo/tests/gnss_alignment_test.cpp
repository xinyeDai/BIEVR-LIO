#include "bievr_pgo/gnss_optimizer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
using Optimizer = bievr::GnssTrajectoryOptimizer;

void require(bool condition, const char* message) {
  if (condition) return;
  std::cerr << message << '\n';
  std::exit(EXIT_FAILURE);
}

void checkAlignment(double late_drift, bool reverse_fixes, bool height_outlier) {
  Optimizer::Config config;
  config.max_time_gap_s = 0.6;
  config.initial_alignment_duration_s = 9.0;
  config.control_pose_interval_s = 0.0;
  config.postfit_outlier_chi2 = 0.0;
  config.gnss_lever_arm_body_m = bievr::V3(1.0, 0.2, 2.0);
  const bievr::Rotation rotation =
      Eigen::AngleAxisd(0.4, bievr::V3::UnitZ()).toRotationMatrix();
  const bievr::Transform expected(rotation, bievr::V3(10.0, -4.0, 3.0));
  std::vector<Optimizer::StampedPose> lio;
  std::vector<Optimizer::PositionMeasurement> fixes;
  for (int i = 0; i <= 60; ++i) {
    const uint64_t stamp = static_cast<uint64_t>(100 + i) * 1000000000ULL;
    const bievr::V3 true_position(i, 0.0, 0.0);
    const double drift = i <= 10 ? 0.0 : late_drift * (i - 10) / 50.0;
    lio.push_back({stamp, bievr::Transform(bievr::Rotation(bievr::Rotation::Identity()),
                                        bievr::V3(true_position + bievr::V3(0, 0, drift)))});
    if (i == 0) continue;
    bievr::V3 antenna = expected * (true_position + config.gnss_lever_arm_body_m);
    if (height_outlier && i == 5) antenna.z() += 100.0;
    fixes.push_back({stamp, antenna, bievr::V3::Ones()});
  }
  if (reverse_fixes) std::reverse(fixes.begin(), fixes.end());
  std::vector<Optimizer::StampedPose> optimized;
  Optimizer::Stats stats;
  std::string message;
  const bool success = Optimizer(config).optimize(lio, fixes, optimized, stats, &message);
  require(success, message.c_str());
  require((stats.initial_T_enu_odom.matrix() - expected.matrix()).norm() < 1e-9,
          "late drift or a vertical outlier biased the early alignment");
  require(stats.initial_alignment_measurements == 10, "incorrect window size");
  require(std::abs(stats.initial_alignment_start_offset_s - 1.0) < 1e-9,
          "incorrect alignment start");
  require(std::abs(stats.initial_alignment_span_s - 9.0) < 1e-9,
          "incorrect alignment span");
  require(optimized.size() == lio.size(), "dense output was truncated to the window");
  require(stats.used_measurements == fixes.size(),
          "later GNSS fixes were excluded from the optimization");
}

void checkInvalidWindow() {
  std::vector<Optimizer::StampedPose> lio;
  std::vector<Optimizer::PositionMeasurement> fixes;
  for (int i = 0; i <= 5; ++i) {
    const uint64_t stamp = static_cast<uint64_t>(100 + i) * 1000000000ULL;
    lio.push_back({stamp, bievr::Transform::Identity()});
    if (i) fixes.push_back({stamp, bievr::V3::Zero(), bievr::V3::Ones()});
  }
  Optimizer::Config config;
  config.max_time_gap_s = 0.6;
  std::vector<Optimizer::StampedPose> output;
  Optimizer::Stats stats;
  std::string message;
  require(!Optimizer(config).optimize(lio, fixes, output, stats, &message) &&
              message.find("horizontal motion") != std::string::npos,
          "stationary window should not yield a yaw seed");
  config.initial_alignment_duration_s = 0.5;
  require(!Optimizer(config).optimize(lio, fixes, output, stats, &message) &&
              message.find("Fewer than three") != std::string::npos,
          "too few early fixes should not fall back to the entire trajectory");
  config.initial_alignment_duration_s = std::numeric_limits<double>::quiet_NaN();
  require(!Optimizer(config).optimize(lio, fixes, output, stats, &message),
          "nonfinite duration should be rejected");
}

void checkLateHorizontalDrift() {
  Optimizer::Config config;
  config.max_time_gap_s = 0.6;
  config.initial_alignment_duration_s = 9.0;
  config.control_pose_interval_s = 0.0;
  // Isolate the seed gate: this regression concerns admission to the first
  // solve, independently of the fit quality and second-stage rejection.
  config.postfit_outlier_chi2 = 0.0;
  std::vector<Optimizer::StampedPose> lio;
  std::vector<Optimizer::PositionMeasurement> fixes;
  for (int i = 0; i <= 100; ++i) {
    const uint64_t stamp = static_cast<uint64_t>(100 + i) * 1000000000ULL;
    const double drift_time = std::max(0, i - 80);
    const double drift = 0.1 * drift_time * drift_time;
    lio.push_back({stamp, bievr::Transform(bievr::Rotation::Identity(),
                                        bievr::V3(i, drift, 0.0))});
    if (i == 0) continue;  // A fix needs a preceding pose for interpolation.
    fixes.push_back({stamp, bievr::V3(i, 0.0, 0.0), bievr::V3::Ones()});
  }
  // GNSS ordering must not determine which fixes belong to the seed window.
  std::reverse(fixes.begin(), fixes.end());
  std::vector<Optimizer::StampedPose> optimized;
  Optimizer::Stats stats;
  std::string message;
  require(Optimizer(config).optimize(lio, fixes, optimized, stats, &message),
          message.c_str());
  require(stats.initial_T_enu_odom.matrix().isApprox(
              bievr::Transform::Identity().matrix(), 1e-9),
          "late horizontal drift biased the early alignment");
  require(stats.effective_initial_outlier_gate_m < 40.0,
          "late-drift regression did not exceed the seed outlier gate");
  if (stats.rejected_outlier != 0 || stats.used_measurements != fixes.size()) {
    std::cerr << "late-drift seed gate=" << stats.effective_initial_outlier_gate_m
              << ", initial rejects=" << stats.rejected_outlier
              << ", postfit rejects=" << stats.rejected_postfit
              << ", retained=" << stats.used_measurements << '/'
              << fixes.size() << '\n';
  }
  require(stats.rejected_outlier == 0 && stats.used_measurements == fixes.size(),
          "normal late GNSS fixes were permanently gated because of LIO drift");
  require(optimized.size() == lio.size(),
          "late-drift optimization truncated the dense trajectory");
}

void checkInitialSeedGate() {
  Optimizer::Config config;
  config.max_time_gap_s = 0.6;
  config.initial_alignment_duration_s = 9.0;
  config.control_pose_interval_s = 0.0;
  config.postfit_outlier_chi2 = 0.0;
  std::vector<Optimizer::StampedPose> lio;
  std::vector<Optimizer::PositionMeasurement> fixes;
  for (int i = 0; i <= 20; ++i) {
    const uint64_t stamp = static_cast<uint64_t>(100 + i) * 1000000000ULL;
    lio.push_back({stamp, bievr::Transform(bievr::Rotation::Identity(),
                                        bievr::V3(i, 0.0, 0.0))});
    if (i == 0) continue;
    fixes.push_back({stamp, bievr::V3(i + (i == 5 ? 20.0 : 0.0), 0.0, 0.0),
                     bievr::V3::Ones()});
  }
  std::vector<Optimizer::StampedPose> optimized;
  Optimizer::Stats stats;
  std::string message;
  require(Optimizer(config).optimize(lio, fixes, optimized, stats, &message),
          message.c_str());
  require(stats.rejected_outlier == 1 &&
              stats.used_measurements == fixes.size() - 1,
          "initial seed gate no longer rejects an isolated early GNSS outlier");

  config.initial_outlier_gate_m = 0.0;
  require(Optimizer(config).optimize(lio, fixes, optimized, stats, &message),
          "zero initial gate must disable seed rejection");
  require(stats.rejected_outlier == 0 && stats.used_measurements == fixes.size() &&
              stats.effective_initial_outlier_gate_m == 0.0,
          "zero initial gate still rejected a seed measurement");
  for (const double invalid_gate : {-1.0,
          std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity()}) {
    config.initial_outlier_gate_m = invalid_gate;
    require(!Optimizer(config).optimize(lio, fixes, optimized, stats, &message),
            "negative or nonfinite initial gate must be rejected");
  }
}
}  // namespace

int main() {
  checkAlignment(0.0, false, false);
  checkAlignment(20.0, true, false);
  checkAlignment(-20.0, false, true);
  checkInvalidWindow();
  checkLateHorizontalDrift();
  checkInitialSeedGate();
  std::cout << "GNSS early alignment tests passed\n";
}
