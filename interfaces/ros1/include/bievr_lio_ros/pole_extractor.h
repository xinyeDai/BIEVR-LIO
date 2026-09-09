#ifndef BIEVR_LIO_ROS_POLE_EXTRACTOR_H_
#define BIEVR_LIO_ROS_POLE_EXTRACTOR_H_

#include <bievr_lio/pipeline.h>
#include <ros/ros.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace bievr {

// Online counterpart of scripts/coarse_extraction.py. Consecutive LIO frames are
// accumulated in the world frame and processed as one pole-extraction keyframe.
struct PoleExtractionConfig {
  bool enable = false;

  // Keyframe construction and background work queue.
  size_t frames_per_keyframe = 50;
  bool process_partial_keyframe = true;
  size_t min_partial_frames = 10;
  size_t max_queue = 3;
  double input_voxel_size = 0.10;

  // Track-relative region of interest.
  bool use_roi = true;
  double roi_y_min = -10.0;
  double roi_y_max = 10.0;
  double roi_z_min = -5.0;
  double roi_z_max = 10.0;

  // 2-D grid and vertical occupancy tests.
  double grid_resolution = 0.5;
  int min_points_per_cell = 60;
  double min_height_span = 8.0;
  double layer_resolution = 0.2;
  int min_continuous_layers = 40;
  int max_component_cells = 8;

  // Euclidean refinement. This corresponds to CLUSTERING_METHOD=EUCLIDEAN in
  // the original script.
  double cluster_tolerance = 1.0;
  double max_xy_span = 1.0;
  int min_cluster_points = 50;
  int max_cluster_points = 5000;

  // SE-LIO-inspired cylinder validation after coarse candidate extraction.
  bool cylinder_fit_enable = true;
  int cylinder_ransac_iterations = 128;
  double cylinder_inlier_threshold = 0.10;
  double cylinder_min_inlier_ratio = 0.35;
  double cylinder_max_rms = 0.20;
  double cylinder_min_radius = 0.08;
  double cylinder_max_radius = 0.80;
  double cylinder_max_axis_tilt_deg = 5.0;
  double cylinder_min_angular_coverage = 0.25;
  int cylinder_max_piecewise_depth = 0;
  int cylinder_min_segment_points = 80;

  // The fitted axis is intersected with trajectory_z + this offset.
  double center_z_offset = -5.0;

  // Merge repeated observations using position, cylinder axis and radius.
  double landmark_merge_radius = 1.0;
  double landmark_max_axis_angle_deg = 5.0;
  double landmark_max_radius_difference = 0.30;

  std::string output_directory = "Log/poles";
  bool save_keyframe_clouds = true;
};

struct PoleExtractionStats {
  size_t received_frames = 0;
  size_t processed_keyframes = 0;
  size_t dropped_keyframes = 0;
  size_t detections = 0;
  size_t landmarks = 0;
};

// Reads pole_extraction: from the same layered YAML files as the LIO config.
bool loadPoleExtractionConfig(const std::vector<std::string>& yaml_paths,
                              PoleExtractionConfig& config);

class PoleExtractor {
 public:
  PoleExtractor(ros::NodeHandle& nh, const std::shared_ptr<Pipeline>& pipeline,
                PoleExtractionConfig config, const std::string& topic_prefix);
  ~PoleExtractor();

  PoleExtractor(const PoleExtractor&) = delete;
  PoleExtractor& operator=(const PoleExtractor&) = delete;

  // Stops accepting frames, optionally queues the final partial keyframe, and
  // drains the worker. Safe to call more than once.
  void finish();

  // Writes landmarks.txt, detections.txt, landmarks.pcd and summary.yaml.
  bool saveResults(std::string* message = nullptr) const;

  PoleExtractionStats stats() const;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_ROS_POLE_EXTRACTOR_H_
