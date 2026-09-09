#ifndef BIEVR_PGO_GNSS_OPTIMIZER_H_
#define BIEVR_PGO_GNSS_OPTIMIZER_H_

#include <bievr_lio/common.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace bievr {

// A batch pose graph that keeps LIO relative motion and anchors its positions
// with GNSS. GNSS samples may lie between LIO poses; the corresponding factor
// constrains the linearly interpolated antenna position instead of snapping a
// measurement to the nearest LiDAR frame.
class GnssTrajectoryOptimizer {
 public:
  struct StampedPose {
    uint64_t stamp = 0;
    Transform pose;
  };

  struct PositionMeasurement {
    uint64_t stamp = 0;
    V3 position = V3::Zero();  // local ENU [m]
    V3 variance = V3::Ones();  // ENU diagonal [m^2]
  };

  struct Config {
    double max_time_gap_s = 0.20;
    double horizontal_sigma_floor_m = 0.05;
    double vertical_sigma_floor_m = 0.10;
    double gnss_huber_delta = 2.5;  // normalized residual units
    double initial_outlier_gate_m = 3.0;

    // Per sqrt(m) LIO relative-motion noise. A short stationary edge still
    // receives the minimum edge scale below to avoid an unrealistically rigid
    // graph during stops.
    double odom_rotation_sigma_deg_per_sqrt_m = 0.02;
    double odom_translation_sigma_m_per_sqrt_m = 0.02;
    double minimum_edge_distance_m = 0.10;
    double prior_roll_pitch_sigma_deg = 1.0;
    double prior_yaw_sigma_deg = 5.0;
    double prior_translation_sigma_m = 100.0;
    int max_iterations = 100;
  };

  struct Stats {
    size_t input_poses = 0;
    size_t input_measurements = 0;
    size_t used_measurements = 0;
    size_t rejected_time = 0;
    size_t rejected_outlier = 0;
    double initial_horizontal_rmse_m = 0.0;
    double optimized_horizontal_rmse_m = 0.0;
    double initial_3d_rmse_m = 0.0;
    double optimized_3d_rmse_m = 0.0;
    double initial_error = 0.0;
    double final_error = 0.0;
    Transform initial_T_enu_odom;
  };

  GnssTrajectoryOptimizer();
  explicit GnssTrajectoryOptimizer(Config config);

  bool optimize(const std::vector<StampedPose>& lio,
                const std::vector<PositionMeasurement>& gnss,
                std::vector<StampedPose>& optimized, Stats& stats,
                std::string* message = nullptr) const;

 private:
  Config config_;
};

}  // namespace bievr

#endif  // BIEVR_PGO_GNSS_OPTIMIZER_H_
