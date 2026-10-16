#ifndef BIEVR_LIO_ROS_POLE_LOCALIZATION_H_
#define BIEVR_LIO_ROS_POLE_LOCALIZATION_H_

#include <bievr_lio/pipeline.h>
#include <bievr_pgo/sparse_localization_graph.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "bievr_lio_ros/pole_extractor.h"

namespace bievr {

// Configuration for localization that uses only LIO odometry and a sparse pole
// map. It deliberately does not load a dense point-cloud map or feed corrections
// back into the LIO estimator.
struct PoleLocalizationConfig {
  bool enable = false;
  std::string map_frame = "map";
  std::string output_directory = "Log/pole_localization";

  bool filter_enable = true;
  bool factor_graph_enable = true;
  double factor_graph_huber_delta = 2.5;
  double factor_graph_prior_translation_sigma_m = 0.50;
  double factor_graph_prior_rotation_sigma_deg = 2.0;
  // Railway LIO can accumulate metre-level horizontal drift over one
  // kilometre. These values let a high-quality multi-pole correction track
  // that slow drift without weakening the matcher's geometric gates.
  double process_noise_xy_m_per_sqrt_m = 0.05;
  double process_noise_yaw_deg_per_sqrt_m = 0.03;
  double process_noise_z_m_per_sqrt_m = 0.05;
  double process_noise_tilt_deg_per_sqrt_m = 0.03;
  double measurement_xy_floor_m = 0.08;
  double measurement_yaw_floor_deg = 0.05;
  double measurement_z_floor_m = 0.08;
  double measurement_tilt_floor_deg = 0.10;
  double mahalanobis_gate = 36.0;
  double max_innovation_translation_m = 5.0;
  double max_innovation_yaw_deg = 2.0;
  double max_innovation_z_m = 1.0;
  double max_innovation_tilt_deg = 2.0;
  double rail_mahalanobis_gate = 36.0;
  // After this much travel without an accepted rail update, a good rail fit
  // is treated as a fresh vertical fix instead of a small tracking update.
  double rail_rebootstrap_gap_m = 100.0;

  int initial_confirmations = 2;
  int recovery_confirmations = 2;
  double degraded_after_distance_m = 90.0;
  double lost_after_distance_m = 240.0;

  bool publish_path = true;
  double path_publish_frequency = 2.0;
  size_t path_max_poses = 0;
  size_t prelocalization_buffer_frames = 5000;
};

bool loadPoleLocalizationConfig(const std::vector<std::string>& yaml_paths,
                                PoleLocalizationConfig& config);

class PoleLocalization {
 public:
  enum class Status { Uninitialized, Searching, Tracking, Degraded, Lost };

  struct Stats {
    size_t measurements = 0;
    size_t accepted = 0;
    size_t rejected = 0;
    size_t localized_frames = 0;
    size_t rail_measurements = 0;
    size_t rail_accepted = 0;
    size_t rail_rejected = 0;
    double last_innovation_m = 0.0;
    double last_innovation_yaw_deg = 0.0;
    double last_innovation_z_m = 0.0;
    double last_innovation_tilt_deg = 0.0;
  };

  PoleLocalization(ros::NodeHandle& nh, const std::shared_ptr<Pipeline>& pipeline,
                   PoleLocalizationConfig config, const std::string& topic_prefix);
  ~PoleLocalization() = default;

  PoleLocalization(const PoleLocalization&) = delete;
  PoleLocalization& operator=(const PoleLocalization&) = delete;

  // Returns a seed even while SEARCHING, allowing the pole matcher to refine a
  // manual /initialpose before this class declares localization TRACKING.
  std::optional<Transform> correction() const;
  void addKeyframe(uint64_t stamp, const Transform& T_odom_body,
                   double odom_distance_m);
  void addPoleCorrection(const PoleLocalizationCorrection& correction);

  Status status() const;
  Stats stats() const;
  bool saveResults(std::string* message = nullptr) const;

  static const char* toString(Status status);

 private:
  void addOdometry(uint64_t stamp, const Transform& T_odom_body);
  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message);
  void publishStatus(Status status);
  Transform correctionLocked() const;
  void syncStateFromCorrectionLocked(const Transform& correction);
  void rewriteTrajectoryFromGraphLocked();
  void addRailCorrection(const PoleLocalizationCorrection& correction);
  void rewriteTrajectoryTailLocked(uint64_t stamp, const Transform& correction);

  struct UpdateRecord {
    uint64_t stamp = 0;
    Transform raw;
    Transform filtered;
    bool accepted = false;
    Status status = Status::Uninitialized;
    size_t matches = 0;
    double match_ratio = 0.0;
    double baseline = 0.0;
    double rms = 0.0;
    double innovation_m = 0.0;
    double innovation_yaw_deg = 0.0;
    double mahalanobis = 0.0;
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  };

  struct BufferedOdometry {
    uint64_t stamp = 0;
    Transform pose;
  };

  PoleLocalizationConfig config_;
  std::unique_ptr<SparseLocalizationGraph> graph_;
  std::string odom_frame_;
  std::string body_frame_;

  mutable std::mutex mutex_;
  bool initialized_ = false;
  Eigen::Vector3d state_ = Eigen::Vector3d::Zero();  // tx, ty, yaw
  Eigen::Matrix3d covariance_ = Eigen::Matrix3d::Identity();
  Rotation fixed_roll_pitch_ = Rotation::Identity();
  double fixed_z_ = 0.0;
  // z, roll and pitch of map->odom. This is updated only by rail-vector
  // measurements; pole updates continue to affect the planar state only.
  Eigen::Vector3d vertical_state_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d vertical_covariance_ = Eigen::Matrix3d::Identity();
  // The configured/map seed initializes the planar search, but its vertical
  // component can be tens of metres wrong after a long LIO run.  The first
  // geometrically verified rail observation therefore bootstraps this state;
  // subsequent observations use the normal innovation gates.
  bool have_rail_vertical_fix_ = false;
  double last_rail_fix_distance_m_ = 0.0;
  Status status_ = Status::Uninitialized;
  int confirmations_ = 0;

  bool have_odom_ = false;
  uint64_t latest_odom_stamp_ = 0;
  Transform latest_odom_;
  V3 last_odom_position_ = V3::Zero();
  double odom_distance_m_ = 0.0;
  bool have_filter_distance_ = false;
  double filter_distance_m_ = 0.0;
  bool have_vertical_filter_distance_ = false;
  double vertical_filter_distance_m_ = 0.0;
  bool have_accepted_measurement_distance_ = false;
  double last_accepted_measurement_distance_m_ = 0.0;
  double distance_since_measurement_ = 0.0;
  uint64_t last_recorded_stamp_ = 0;
  uint64_t last_path_publish_stamp_ = 0;

  Path trajectory_;
  Path path_;
  std::vector<BufferedOdometry> prelocalization_buffer_;
  // Source poses parallel to trajectory_. They allow a delayed background
  // pole update to correct poses from its sensor timestamp onward.
  std::vector<BufferedOdometry> localized_odometry_;
  std::vector<UpdateRecord> updates_;
  Stats stats_;

  ros::Publisher odom_pub_;
  ros::Publisher path_pub_;
  ros::Publisher status_pub_;
  ros::Publisher correction_pub_;
  ros::Subscriber initial_pose_sub_;
  tf::TransformBroadcaster tf_;
};

}  // namespace bievr

#endif  // BIEVR_LIO_ROS_POLE_LOCALIZATION_H_
