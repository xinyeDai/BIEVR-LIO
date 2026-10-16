#ifndef BIEVR_PGO_SPARSE_LOCALIZATION_GRAPH_H_
#define BIEVR_PGO_SPARSE_LOCALIZATION_GRAPH_H_

#include <bievr_lio/common.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace bievr {

// Incremental Pose3 graph used by sparse pole/rail localization. Every
// extraction keyframe is a graph variable, LIO supplies the relative-motion
// factors, pole observations constrain x/y/yaw, and rail observations constrain
// cross-track/z/roll/pitch/yaw while leaving along-track translation free.
// GTSAM is kept behind a pimpl so ROS consumers do not need GTSAM types in
// their public interfaces.
class SparseLocalizationGraph {
 public:
  struct Config {
    double odom_xy_sigma_m_per_sqrt_m = 0.05;
    double odom_z_sigma_m_per_sqrt_m = 0.05;
    double odom_yaw_sigma_deg_per_sqrt_m = 0.03;
    double odom_tilt_sigma_deg_per_sqrt_m = 0.03;
    double minimum_edge_distance_m = 0.10;
    double prior_translation_sigma_m = 0.50;
    double prior_rotation_sigma_deg = 2.0;
    double robust_huber_delta = 2.5;  // normalized residual units
    double isam_relinearize_threshold = 0.01;
    int isam_relinearize_skip = 1;
  };

  struct KeyframeEstimate {
    uint64_t stamp = 0;
    Transform odom_pose;
    Transform map_pose;
  };

  SparseLocalizationGraph();
  explicit SparseLocalizationGraph(Config config);
  ~SparseLocalizationGraph();

  SparseLocalizationGraph(const SparseLocalizationGraph&) = delete;
  SparseLocalizationGraph& operator=(const SparseLocalizationGraph&) = delete;

  // Resets the graph gauge from a manual/configured map<-odom seed while
  // retaining keyframes already received from the extraction worker.
  void setInitialCorrection(const Transform& T_map_odom);

  // Keyframes must be supplied in increasing timestamp order. Duplicate stamps
  // are ignored, which also makes replayed worker callbacks harmless.
  bool addKeyframe(uint64_t stamp, const Transform& T_odom_body);

  // Measurements are aggregate front-end registrations expressed as
  // map<-odom. Internally they are converted to an absolute observation of the
  // Pose3 at `stamp`, then only the stated components become residuals.
  bool addPlanarMeasurement(uint64_t stamp, const Transform& T_map_odom,
                            double xy_sigma_m, double yaw_sigma_rad);
  bool addVerticalMeasurement(uint64_t stamp, const Transform& T_map_odom,
                              double z_sigma_m, double roll_sigma_rad,
                              double pitch_sigma_rad);
  bool addRailMeasurement(uint64_t stamp, const Transform& T_map_odom,
                          const V3& tangent_map, double cross_track_sigma_m,
                          double z_sigma_m, double roll_sigma_rad,
                          double pitch_sigma_rad, double yaw_sigma_rad);

  bool initialized() const;
  size_t size() const;
  std::optional<Transform> correctionAt(uint64_t stamp) const;
  std::vector<KeyframeEstimate> estimates() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bievr

#endif  // BIEVR_PGO_SPARSE_LOCALIZATION_GRAPH_H_
