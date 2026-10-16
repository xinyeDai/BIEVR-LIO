#include "bievr_lio_ros/pole_localization.h"

#include <bievr_lio/config_loader.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/utils.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <utility>

#include "bievr_ros_common/conversions.h"

namespace bievr {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;

double wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double yawOf(const Rotation& rotation) {
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

Eigen::Vector3d rpyOf(const Rotation& rotation) {
  const double pitch = std::asin(std::clamp(-rotation(2, 0), -1.0, 1.0));
  const double roll = std::atan2(rotation(2, 1), rotation(2, 2));
  return Eigen::Vector3d(roll, pitch, yawOf(rotation));
}

Rotation rpyRotation(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, V3::UnitZ()) *
          Eigen::AngleAxisd(pitch, V3::UnitY()) *
          Eigen::AngleAxisd(roll, V3::UnitX())).toRotationMatrix();
}

Rotation yawRotation(double yaw) {
  return Eigen::AngleAxisd(yaw, V3::UnitZ()).toRotationMatrix();
}

uint64_t secondsToNanoseconds(double seconds) {
  if (seconds <= 0.0) return 0;
  const double value = seconds * 1.0e9;
  if (value >= static_cast<double>(std::numeric_limits<uint64_t>::max())) {
    return std::numeric_limits<uint64_t>::max();
  }
  return static_cast<uint64_t>(value);
}

SparseLocalizationGraph::Config graphConfig(
    const PoleLocalizationConfig& config) {
  SparseLocalizationGraph::Config graph;
  graph.odom_xy_sigma_m_per_sqrt_m =
      config.process_noise_xy_m_per_sqrt_m;
  graph.odom_z_sigma_m_per_sqrt_m = config.process_noise_z_m_per_sqrt_m;
  graph.odom_yaw_sigma_deg_per_sqrt_m =
      config.process_noise_yaw_deg_per_sqrt_m;
  graph.odom_tilt_sigma_deg_per_sqrt_m =
      config.process_noise_tilt_deg_per_sqrt_m;
  graph.prior_translation_sigma_m =
      config.factor_graph_prior_translation_sigma_m;
  graph.prior_rotation_sigma_deg = config.factor_graph_prior_rotation_sigma_deg;
  graph.robust_huber_delta = config.factor_graph_huber_delta;
  return graph;
}

}  // namespace

bool loadPoleLocalizationConfig(const std::vector<std::string>& yaml_paths,
                                PoleLocalizationConfig& config) {
  config_internal::MergedYaml yaml;
  for (const std::string& path : yaml_paths) {
    if (path.empty()) continue;
    try {
      yaml.add(YAML::LoadFile(path));
    } catch (const std::exception& error) {
      LOG(E, "Failed to load YAML config '" << path << "': " << error.what());
      return false;
    }
  }

  const std::string section = "pole_localization";
  config.enable = yaml.get<bool>(section, "enable", config.enable);
  if (!config.enable) return true;

  config.map_frame = yaml.get<std::string>(section, "map_frame", config.map_frame);
  config.output_directory =
      yaml.get<std::string>(section, "output_directory", config.output_directory);
  config.filter_enable = yaml.get<bool>(section, "filter_enable", config.filter_enable);
  config.factor_graph_enable = yaml.get<bool>(
      section, "factor_graph_enable", config.factor_graph_enable);
  config.factor_graph_huber_delta = yaml.get<double>(
      section, "factor_graph_huber_delta", config.factor_graph_huber_delta);
  config.factor_graph_prior_translation_sigma_m = yaml.get<double>(
      section, "factor_graph_prior_translation_sigma_m",
      config.factor_graph_prior_translation_sigma_m);
  config.factor_graph_prior_rotation_sigma_deg = yaml.get<double>(
      section, "factor_graph_prior_rotation_sigma_deg",
      config.factor_graph_prior_rotation_sigma_deg);
  config.process_noise_xy_m_per_sqrt_m = yaml.get<double>(
      section, "process_noise_xy_m_per_sqrt_m", config.process_noise_xy_m_per_sqrt_m);
  config.process_noise_yaw_deg_per_sqrt_m = yaml.get<double>(
      section, "process_noise_yaw_deg_per_sqrt_m",
      config.process_noise_yaw_deg_per_sqrt_m);
  config.process_noise_z_m_per_sqrt_m = yaml.get<double>(
      section, "process_noise_z_m_per_sqrt_m",
      config.process_noise_z_m_per_sqrt_m);
  config.process_noise_tilt_deg_per_sqrt_m = yaml.get<double>(
      section, "process_noise_tilt_deg_per_sqrt_m",
      config.process_noise_tilt_deg_per_sqrt_m);
  config.measurement_xy_floor_m = yaml.get<double>(
      section, "measurement_xy_floor_m", config.measurement_xy_floor_m);
  config.measurement_yaw_floor_deg = yaml.get<double>(
      section, "measurement_yaw_floor_deg", config.measurement_yaw_floor_deg);
  config.measurement_z_floor_m = yaml.get<double>(
      section, "measurement_z_floor_m", config.measurement_z_floor_m);
  config.measurement_tilt_floor_deg = yaml.get<double>(
      section, "measurement_tilt_floor_deg", config.measurement_tilt_floor_deg);
  config.mahalanobis_gate =
      yaml.get<double>(section, "mahalanobis_gate", config.mahalanobis_gate);
  config.max_innovation_translation_m = yaml.get<double>(
      section, "max_innovation_translation_m", config.max_innovation_translation_m);
  config.max_innovation_yaw_deg = yaml.get<double>(
      section, "max_innovation_yaw_deg", config.max_innovation_yaw_deg);
  config.max_innovation_z_m = yaml.get<double>(
      section, "max_innovation_z_m", config.max_innovation_z_m);
  config.max_innovation_tilt_deg = yaml.get<double>(
      section, "max_innovation_tilt_deg", config.max_innovation_tilt_deg);
  config.rail_mahalanobis_gate = yaml.get<double>(
      section, "rail_mahalanobis_gate", config.rail_mahalanobis_gate);
  config.rail_rebootstrap_gap_m = yaml.get<double>(
      section, "rail_rebootstrap_gap_m", config.rail_rebootstrap_gap_m);
  config.initial_confirmations =
      yaml.get<int>(section, "initial_confirmations", config.initial_confirmations);
  config.recovery_confirmations =
      yaml.get<int>(section, "recovery_confirmations", config.recovery_confirmations);
  config.degraded_after_distance_m = yaml.get<double>(
      section, "degraded_after_distance_m", config.degraded_after_distance_m);
  config.lost_after_distance_m =
      yaml.get<double>(section, "lost_after_distance_m", config.lost_after_distance_m);
  config.publish_path = yaml.get<bool>(section, "publish_path", config.publish_path);
  config.path_publish_frequency = yaml.get<double>(
      section, "path_publish_frequency", config.path_publish_frequency);
  const int max_poses = yaml.get<int>(section, "path_max_poses",
                                      static_cast<int>(config.path_max_poses));
  const int buffer_frames =
      yaml.get<int>(section, "prelocalization_buffer_frames",
                    static_cast<int>(config.prelocalization_buffer_frames));

  if (config.map_frame.empty() || config.output_directory.empty() ||
      config.process_noise_xy_m_per_sqrt_m <= 0.0 ||
      config.process_noise_yaw_deg_per_sqrt_m <= 0.0 ||
      config.process_noise_z_m_per_sqrt_m <= 0.0 ||
      config.process_noise_tilt_deg_per_sqrt_m <= 0.0 ||
      config.factor_graph_huber_delta <= 0.0 ||
      config.factor_graph_prior_translation_sigma_m <= 0.0 ||
      config.factor_graph_prior_rotation_sigma_deg <= 0.0 ||
      config.measurement_xy_floor_m <= 0.0 ||
      config.measurement_yaw_floor_deg <= 0.0 ||
      config.measurement_z_floor_m <= 0.0 ||
      config.measurement_tilt_floor_deg <= 0.0 || config.mahalanobis_gate <= 0.0 ||
      config.max_innovation_translation_m <= 0.0 ||
      config.max_innovation_yaw_deg <= 0.0 || config.initial_confirmations <= 0 ||
      config.max_innovation_z_m <= 0.0 || config.max_innovation_tilt_deg <= 0.0 ||
      config.rail_mahalanobis_gate <= 0.0 ||
      config.rail_rebootstrap_gap_m <= 0.0 ||
      config.recovery_confirmations <= 0 || config.degraded_after_distance_m <= 0.0 ||
      config.lost_after_distance_m <= config.degraded_after_distance_m ||
      config.path_publish_frequency <= 0.0 || max_poses < 0 || buffer_frames <= 0) {
    LOG(E, "Config error: invalid pole_localization parameter.");
    return false;
  }
  config.path_max_poses = static_cast<size_t>(max_poses);
  config.prelocalization_buffer_frames = static_cast<size_t>(buffer_frames);
  LOG(I, "Pole-only localization enabled: map frame '" << config.map_frame
                                                         << "', output '"
                                                         << config.output_directory << "'.");
  return true;
}

const char* PoleLocalization::toString(Status status) {
  switch (status) {
    case Status::Uninitialized:
      return "UNINITIALIZED";
    case Status::Searching:
      return "SEARCHING";
    case Status::Tracking:
      return "TRACKING";
    case Status::Degraded:
      return "DEGRADED";
    case Status::Lost:
      return "LOST";
  }
  return "UNKNOWN";
}

PoleLocalization::PoleLocalization(ros::NodeHandle& nh,
                                   const std::shared_ptr<Pipeline>& pipeline,
                                   PoleLocalizationConfig config,
                                   const std::string& topic_prefix)
    : config_(std::move(config)),
      graph_(config_.factor_graph_enable
                 ? std::make_unique<SparseLocalizationGraph>(graphConfig(config_))
                 : nullptr),
      odom_frame_(pipeline->config().map_frame),
      body_frame_(pipeline->config().body_frame) {
  odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_prefix + "/loc/odom", 10, true);
  path_pub_ = nh.advertise<nav_msgs::Path>(topic_prefix + "/loc/path", 1, true);
  status_pub_ = nh.advertise<std_msgs::String>(topic_prefix + "/loc/status", 1, true);
  correction_pub_ =
      nh.advertise<nav_msgs::Odometry>(topic_prefix + "/poles/correction_filtered", 1, true);
  initial_pose_sub_ = nh.subscribe("/initialpose", 1, &PoleLocalization::onInitialPose, this);
  pipeline->addFrameObserver(
      [this](uint64_t stamp, const Transform& pose, const Pointcloud&) {
        addOdometry(stamp, pose);
      });
  publishStatus(Status::Uninitialized);
}

PoleLocalization::Status PoleLocalization::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

PoleLocalization::Stats PoleLocalization::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

Transform PoleLocalization::correctionLocked() const {
  if (graph_ && graph_->initialized()) {
    const uint64_t stamp = have_odom_ ? latest_odom_stamp_ : 0;
    if (const auto correction = graph_->correctionAt(stamp)) return *correction;
  }
  return Transform(rpyRotation(vertical_state_.y(), vertical_state_.z(), state_.z()),
                   V3(state_.x(), state_.y(), vertical_state_.x()));
}

void PoleLocalization::syncStateFromCorrectionLocked(
    const Transform& correction) {
  const Eigen::Vector3d rpy = rpyOf(correction.rotation());
  state_ = Eigen::Vector3d(correction.translation().x(),
                           correction.translation().y(), rpy.z());
  vertical_state_ = Eigen::Vector3d(correction.translation().z(),
                                    rpy.x(), rpy.y());
}

void PoleLocalization::rewriteTrajectoryFromGraphLocked() {
  if (!graph_ || !graph_->initialized()) return;
  for (size_t index = 0; index < localized_odometry_.size(); ++index) {
    const BufferedOdometry& odometry = localized_odometry_[index];
    const auto correction = graph_->correctionAt(odometry.stamp);
    if (!correction) continue;
    trajectory_.poses[index].pose =
        Transform(Eigen::Isometry3d(*correction * odometry.pose));
  }
  for (StampedPose& pose : path_.poses) {
    const auto odometry = std::lower_bound(
        localized_odometry_.begin(), localized_odometry_.end(), pose.stamp,
        [](const BufferedOdometry& candidate, uint64_t stamp) {
          return candidate.stamp < stamp;
        });
    if (odometry == localized_odometry_.end() || odometry->stamp != pose.stamp) continue;
    const auto correction = graph_->correctionAt(pose.stamp);
    if (correction) {
      pose.pose = Transform(Eigen::Isometry3d(*correction * odometry->pose));
    }
  }
}

std::optional<Transform> PoleLocalization::correction() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) return std::nullopt;
  return correctionLocked();
}

void PoleLocalization::addKeyframe(uint64_t stamp,
                                   const Transform& T_odom_body,
                                   double /*odom_distance_m*/) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!graph_) return;
  if (!graph_->addKeyframe(stamp, T_odom_body)) {
    LOG(W, "Ignored out-of-order sparse-localization keyframe at " << stamp << ".");
  }
}

void PoleLocalization::publishStatus(Status status) {
  std_msgs::String message;
  message.data = std::string("POLE_") + toString(status);
  status_pub_.publish(message);
}

void PoleLocalization::onInitialPose(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& message) {
  if (!message->header.frame_id.empty() && message->header.frame_id != config_.map_frame) {
    LOG(W, "Ignoring /initialpose in frame '" << message->header.frame_id
                                               << "'; expected '" << config_.map_frame << "'.");
    return;
  }

  Status new_status = Status::Uninitialized;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_odom_) {
      LOG(W, "Ignoring /initialpose because no LIO odometry has arrived yet.");
      return;
    }
    const auto& position = message->pose.pose.position;
    const auto& orientation = message->pose.pose.orientation;
    const Transform T_map_body(
        Quaternion(orientation.w, orientation.x, orientation.y, orientation.z),
        V3(position.x, position.y, position.z));
    const Transform seed(Eigen::Isometry3d(T_map_body * latest_odom_.inverse()));
    const double yaw = yawOf(seed.rotation());
    state_ = Eigen::Vector3d(seed.translation().x(), seed.translation().y(), yaw);
    fixed_roll_pitch_ = yawRotation(-yaw) * seed.rotation();
    fixed_z_ = seed.translation().z();
    const Eigen::Vector3d rpy = rpyOf(seed.rotation());
    vertical_state_ = Eigen::Vector3d(seed.translation().z(), rpy.x(), rpy.y());
    if (graph_) graph_->setInitialCorrection(seed);

    const auto& input_covariance = message->pose.covariance;
    const double xy_floor_sq =
        config_.measurement_xy_floor_m * config_.measurement_xy_floor_m;
    const double yaw_floor = config_.measurement_yaw_floor_deg * kDegreesToRadians;
    covariance_.setZero();
    covariance_(0, 0) = std::max(xy_floor_sq, input_covariance[0]);
    covariance_(1, 1) = std::max(xy_floor_sq, input_covariance[7]);
    covariance_(2, 2) = std::max(yaw_floor * yaw_floor, input_covariance[35]);
    const double z_floor_sq =
        config_.measurement_z_floor_m * config_.measurement_z_floor_m;
    const double tilt_floor = config_.measurement_tilt_floor_deg * kDegreesToRadians;
    vertical_covariance_.setZero();
    vertical_covariance_(0, 0) = std::max(z_floor_sq, input_covariance[14]);
    vertical_covariance_(1, 1) =
        std::max(tilt_floor * tilt_floor, input_covariance[21]);
    vertical_covariance_(2, 2) =
        std::max(tilt_floor * tilt_floor, input_covariance[28]);
    initialized_ = true;
    have_filter_distance_ = true;
    filter_distance_m_ = odom_distance_m_;
    have_vertical_filter_distance_ = true;
    vertical_filter_distance_m_ = odom_distance_m_;
    have_rail_vertical_fix_ = false;
    last_rail_fix_distance_m_ = odom_distance_m_;
    have_accepted_measurement_distance_ = false;
    confirmations_ = 0;
    distance_since_measurement_ = 0.0;
    status_ = Status::Searching;
    new_status = status_;
  }
  publishStatus(new_status);
  LOG(I, "Pole localization accepted /initialpose and is waiting for pole confirmation.");
}

void PoleLocalization::addPoleCorrection(const PoleLocalizationCorrection& raw) {
  if (!raw.valid || !raw.T_map_odom.matrix().allFinite()) return;
  if (raw.constrain_vertical && !raw.constrain_planar) {
    addRailCorrection(raw);
    return;
  }

  Status old_status = Status::Uninitialized;
  Status new_status = Status::Uninitialized;
  Transform filtered;
  Eigen::Matrix3d filtered_covariance = Eigen::Matrix3d::Identity();
  bool accepted = false;
  double mahalanobis = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.measurements;
    old_status = status_;

    const double raw_yaw = yawOf(raw.T_map_odom.rotation());
    const Eigen::Vector3d measurement(raw.T_map_odom.translation().x(),
                                      raw.T_map_odom.translation().y(), raw_yaw);
    const double quality_ratio = std::sqrt(std::max(0.1, raw.match_ratio));
    const double xy_sigma = std::max(
        config_.measurement_xy_floor_m,
        raw.rms / std::sqrt(static_cast<double>(std::max<size_t>(1, raw.matches))) /
            quality_ratio);
    const double yaw_sigma = std::max(
        config_.measurement_yaw_floor_deg * kDegreesToRadians,
        raw.rms / std::max(raw.baseline, 1.0) / quality_ratio);
    Eigen::Matrix3d measurement_covariance = Eigen::Matrix3d::Zero();
    measurement_covariance(0, 0) = xy_sigma * xy_sigma;
    measurement_covariance(1, 1) = xy_sigma * xy_sigma;
    measurement_covariance(2, 2) = yaw_sigma * yaw_sigma;

    // Corrections are produced by a background extraction thread. Propagate
    // only to the correction's sensor-time distance so offline fast playback
    // and online real-time execution use the same filter chronology.
    if (initialized_ && have_filter_distance_) {
      const double traveled = std::max(0.0, raw.odom_distance_m - filter_distance_m_);
      const double xy_process = config_.process_noise_xy_m_per_sqrt_m;
      const double yaw_process =
          config_.process_noise_yaw_deg_per_sqrt_m * kDegreesToRadians;
      covariance_(0, 0) += xy_process * xy_process * traveled;
      covariance_(1, 1) += xy_process * xy_process * traveled;
      covariance_(2, 2) += yaw_process * yaw_process * traveled;
    }
    filter_distance_m_ = std::max(filter_distance_m_, raw.odom_distance_m);
    have_filter_distance_ = true;

    if (!initialized_) {
      state_ = measurement;
      covariance_ = measurement_covariance;
      fixed_roll_pitch_ = yawRotation(-raw_yaw) * raw.T_map_odom.rotation();
      fixed_z_ = raw.T_map_odom.translation().z();
      const Eigen::Vector3d raw_rpy = rpyOf(raw.T_map_odom.rotation());
      vertical_state_ = Eigen::Vector3d(raw.T_map_odom.translation().z(),
                                        raw_rpy.x(), raw_rpy.y());
      const double z_floor = config_.measurement_z_floor_m;
      const double tilt_floor = config_.measurement_tilt_floor_deg * kDegreesToRadians;
      vertical_covariance_ = Eigen::Vector3d(z_floor * z_floor,
                                             tilt_floor * tilt_floor,
                                             tilt_floor * tilt_floor).asDiagonal();
      have_vertical_filter_distance_ = true;
      vertical_filter_distance_m_ = raw.odom_distance_m;
      initialized_ = true;
      status_ = Status::Searching;
      confirmations_ = 1;
      accepted = true;
    } else {
      // Evaluate translation innovation at the observed pole-window anchor.
      // Raw map<-odom tx/ty are coordinate-origin dependent and therefore are
      // unsuitable filter innovations on a long railway trajectory.
      const Transform current_correction = correctionLocked();
      const double filtered_yaw = yawOf(current_correction.rotation());
      const V3 raw_anchor_map = raw.T_map_odom * raw.correction_anchor_odom;
      const V3 filtered_anchor_map =
          current_correction * raw.correction_anchor_odom;
      Eigen::Vector3d innovation(raw_anchor_map.x() - filtered_anchor_map.x(),
                                 raw_anchor_map.y() - filtered_anchor_map.y(),
                                 raw_yaw - filtered_yaw);
      innovation.z() = wrapAngle(innovation.z());
      stats_.last_innovation_m = innovation.head<2>().norm();
      stats_.last_innovation_yaw_deg = std::abs(innovation.z()) * kRadiansToDegrees;
      const Eigen::Matrix3d innovation_covariance = covariance_ + measurement_covariance;
      const Eigen::LDLT<Eigen::Matrix3d> decomposition(innovation_covariance);
      mahalanobis = decomposition.info() == Eigen::Success
                        ? innovation.dot(decomposition.solve(innovation))
                        : std::numeric_limits<double>::infinity();
      const bool passes_gate =
          stats_.last_innovation_m <= config_.max_innovation_translation_m &&
          stats_.last_innovation_yaw_deg <= config_.max_innovation_yaw_deg &&
          mahalanobis <= config_.mahalanobis_gate;
      if (passes_gate) {
        if (config_.filter_enable) {
          const Eigen::Matrix3d gain =
              covariance_ * decomposition.solve(Eigen::Matrix3d::Identity());
          const Eigen::Vector3d update = gain * innovation;
          const double updated_yaw = wrapAngle(filtered_yaw + update.z());
          const Eigen::Vector2d updated_anchor =
              filtered_anchor_map.head<2>() + update.head<2>();
          const Rotation updated_rotation =
              rpyRotation(vertical_state_.y(), vertical_state_.z(), updated_yaw);
          const V3 rotated_anchor = updated_rotation * raw.correction_anchor_odom;
          state_.x() = updated_anchor.x() - rotated_anchor.x();
          state_.y() = updated_anchor.y() - rotated_anchor.y();
          state_.z() = updated_yaw;
          const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
          const Eigen::Matrix3d residual = identity - gain;
          covariance_ = residual * covariance_ * residual.transpose() +
                        gain * measurement_covariance * gain.transpose();
        } else {
          state_ = measurement;
          covariance_ = measurement_covariance;
        }
        ++confirmations_;
        accepted = true;
      } else {
        ++stats_.rejected;
        confirmations_ = 0;
        LOG(W, "Rejected filtered pole update: innovation "
                   << stats_.last_innovation_m << " m / "
                   << stats_.last_innovation_yaw_deg << " deg, Mahalanobis "
                   << mahalanobis << ".");
      }
    }

    if (accepted && graph_) {
      if (graph_->addPlanarMeasurement(raw.stamp, raw.T_map_odom, xy_sigma,
                                       yaw_sigma)) {
        if (const auto optimized = graph_->correctionAt(raw.stamp)) {
          syncStateFromCorrectionLocked(*optimized);
        }
      } else {
        LOG(W, "Could not attach pole factor to keyframe at " << raw.stamp
                                                                << ".");
      }
    }

    if (accepted) {
      ++stats_.accepted;
      have_accepted_measurement_distance_ = true;
      last_accepted_measurement_distance_m_ = raw.odom_distance_m;
      distance_since_measurement_ =
          std::max(0.0, odom_distance_m_ - last_accepted_measurement_distance_m_);
      const int required =
          (old_status == Status::Uninitialized || old_status == Status::Searching)
              ? config_.initial_confirmations
              : config_.recovery_confirmations;
      if (confirmations_ >= required) status_ = Status::Tracking;
    }
    if (have_accepted_measurement_distance_) {
      distance_since_measurement_ =
          std::max(0.0, odom_distance_m_ - last_accepted_measurement_distance_m_);
      if (status_ == Status::Tracking &&
          distance_since_measurement_ >= config_.degraded_after_distance_m) {
        status_ = Status::Degraded;
        confirmations_ = 0;
      }
      if ((status_ == Status::Tracking || status_ == Status::Degraded) &&
          distance_since_measurement_ >= config_.lost_after_distance_m) {
        status_ = Status::Lost;
        confirmations_ = 0;
      }
    }
    new_status = status_;
    filtered = correctionLocked();
    filtered_covariance = covariance_;
    if (new_status == Status::Tracking &&
        (old_status == Status::Uninitialized || old_status == Status::Searching)) {
      // The extraction worker can finish a keyframe after process_bag has
      // already delivered newer odometry. Backfill from the confirming sensor
      // timestamp so the saved trajectory is independent of thread timing.
      for (const BufferedOdometry& odometry : prelocalization_buffer_) {
        if (odometry.stamp < raw.stamp || odometry.stamp <= last_recorded_stamp_) continue;
        const Transform pose(Eigen::Isometry3d(filtered * odometry.pose));
        trajectory_.poses.push_back({odometry.stamp, pose});
        path_.poses.push_back({odometry.stamp, pose});
        localized_odometry_.push_back(odometry);
        last_recorded_stamp_ = odometry.stamp;
        ++stats_.localized_frames;
      }
      if (config_.path_max_poses > 0 && path_.poses.size() > config_.path_max_poses) {
        path_.poses.erase(path_.poses.begin(),
                          path_.poses.begin() +
                              static_cast<std::ptrdiff_t>(path_.poses.size() -
                                                          config_.path_max_poses));
      }
      prelocalization_buffer_.clear();
    }
    const bool has_confirmed_fix = status_ == Status::Tracking ||
                                   status_ == Status::Degraded ||
                                   status_ == Status::Lost;
    if (accepted && has_confirmed_fix) {
      if (graph_ && graph_->initialized()) {
        rewriteTrajectoryFromGraphLocked();
      } else {
        // Apply a delayed correction at its actual sensor timestamp. Corrections
        // arrive in keyframe order, so each pass only replaces the trajectory
        // tail governed by the new map->odom estimate.
        const auto begin = std::lower_bound(
            localized_odometry_.begin(), localized_odometry_.end(), raw.stamp,
            [](const BufferedOdometry& odometry, uint64_t stamp) {
              return odometry.stamp < stamp;
            });
        for (auto iterator = begin; iterator != localized_odometry_.end(); ++iterator) {
          const size_t index =
              static_cast<size_t>(iterator - localized_odometry_.begin());
          trajectory_.poses[index].pose =
              Transform(Eigen::Isometry3d(filtered * iterator->pose));
        }
        for (StampedPose& pose : path_.poses) {
          if (pose.stamp < raw.stamp) continue;
          const auto odometry = std::lower_bound(
              localized_odometry_.begin(), localized_odometry_.end(), pose.stamp,
              [](const BufferedOdometry& candidate, uint64_t stamp) {
                return candidate.stamp < stamp;
              });
          if (odometry != localized_odometry_.end() &&
              odometry->stamp == pose.stamp) {
            pose.pose = Transform(Eigen::Isometry3d(filtered * odometry->pose));
          }
        }
      }
    }
    updates_.push_back({raw.stamp,
                        raw.T_map_odom,
                        filtered,
                        accepted,
                        status_,
                        raw.matches,
                        raw.match_ratio,
                        raw.baseline,
                        raw.rms,
                        stats_.last_innovation_m,
                        stats_.last_innovation_yaw_deg,
                        mahalanobis,
                        covariance_});
  }

  if (new_status != old_status) {
    publishStatus(new_status);
    LOG(I, "Pole localization status: " << toString(new_status) << ".");
  }
  if (!accepted) return;

  nav_msgs::Odometry message;
  message.header.stamp.fromNSec(raw.stamp);
  message.header.frame_id = config_.map_frame;
  message.child_frame_id = odom_frame_;
  transformToMsg(filtered, message.pose.pose);
  message.pose.covariance[0] = filtered_covariance(0, 0);
  message.pose.covariance[7] = filtered_covariance(1, 1);
  message.pose.covariance[14] = 1.0e3;
  message.pose.covariance[21] = 1.0e3;
  message.pose.covariance[28] = 1.0e3;
  message.pose.covariance[35] = filtered_covariance(2, 2);
  correction_pub_.publish(message);
}

void PoleLocalization::rewriteTrajectoryTailLocked(
    uint64_t stamp, const Transform& correction) {
  const auto begin = std::lower_bound(
      localized_odometry_.begin(), localized_odometry_.end(), stamp,
      [](const BufferedOdometry& odometry, uint64_t value) {
        return odometry.stamp < value;
      });
  for (auto iterator = begin; iterator != localized_odometry_.end(); ++iterator) {
    const size_t index = static_cast<size_t>(iterator - localized_odometry_.begin());
    trajectory_.poses[index].pose =
        Transform(Eigen::Isometry3d(correction * iterator->pose));
  }
  for (StampedPose& pose : path_.poses) {
    if (pose.stamp < stamp) continue;
    const auto odometry = std::lower_bound(
        localized_odometry_.begin(), localized_odometry_.end(), pose.stamp,
        [](const BufferedOdometry& candidate, uint64_t value) {
          return candidate.stamp < value;
        });
    if (odometry != localized_odometry_.end() && odometry->stamp == pose.stamp) {
      pose.pose = Transform(Eigen::Isometry3d(correction * odometry->pose));
    }
  }
}

void PoleLocalization::addRailCorrection(
    const PoleLocalizationCorrection& raw) {
  Transform filtered;
  Eigen::Matrix3d filtered_covariance = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d planar_covariance = Eigen::Matrix3d::Identity();
  bool accepted = false;
  double mahalanobis = std::numeric_limits<double>::infinity();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.rail_measurements;
    if (!initialized_) {
      ++stats_.rail_rejected;
      return;
    }
    const Eigen::Vector3d rpy = rpyOf(raw.T_map_odom.rotation());
    Eigen::Vector3d measurement(raw.T_map_odom.translation().z(),
                                rpy.x(), rpy.y());
    if (!raw.constrain_z) measurement.x() = vertical_state_.x();
    if (!raw.constrain_roll) measurement.y() = vertical_state_.y();
    if (!raw.constrain_pitch) measurement.z() = vertical_state_.z();
    const double quality = std::sqrt(static_cast<double>(
        std::max<size_t>(1, raw.matches)));
    const double z_sigma = std::max(config_.measurement_z_floor_m,
                                    raw.rms / quality);
    const double tilt_sigma = std::max(
        config_.measurement_tilt_floor_deg * kDegreesToRadians,
        raw.rms / std::max(raw.baseline, 1.0));
    const double cross_track_sigma = std::max(
        config_.measurement_xy_floor_m,
        raw.planar_rms / quality);
    const double rail_yaw_sigma = std::max(
        config_.measurement_yaw_floor_deg * kDegreesToRadians,
        raw.planar_rms / std::max(raw.baseline, 1.0));
    Eigen::Matrix3d measurement_covariance = Eigen::Vector3d(
        z_sigma * z_sigma, tilt_sigma * tilt_sigma,
        tilt_sigma * tilt_sigma).asDiagonal();
    constexpr double kUnconstrainedVariance = 1.0e12;
    if (!raw.constrain_z) measurement_covariance(0, 0) = kUnconstrainedVariance;
    if (!raw.constrain_roll) measurement_covariance(1, 1) = kUnconstrainedVariance;
    if (!raw.constrain_pitch) measurement_covariance(2, 2) = kUnconstrainedVariance;

    if (have_vertical_filter_distance_) {
      const double traveled = std::max(
          0.0, raw.odom_distance_m - vertical_filter_distance_m_);
      const double z_process = config_.process_noise_z_m_per_sqrt_m;
      const double tilt_process =
          config_.process_noise_tilt_deg_per_sqrt_m * kDegreesToRadians;
      vertical_covariance_(0, 0) += z_process * z_process * traveled;
      vertical_covariance_(1, 1) += tilt_process * tilt_process * traveled;
      vertical_covariance_(2, 2) += tilt_process * tilt_process * traveled;
    }
    have_vertical_filter_distance_ = true;
    vertical_filter_distance_m_ =
        std::max(vertical_filter_distance_m_, raw.odom_distance_m);

    const Transform current_correction = correctionLocked();
    const Eigen::Vector3d current_rpy = rpyOf(current_correction.rotation());
    const V3 raw_anchor_map = raw.T_map_odom * raw.correction_anchor_odom;
    const V3 filtered_anchor_map =
        current_correction * raw.correction_anchor_odom;
    Eigen::Vector3d innovation(raw_anchor_map.z() - filtered_anchor_map.z(),
                               measurement.y() - current_rpy.x(),
                               measurement.z() - current_rpy.y());
    innovation.y() = wrapAngle(innovation.y());
    innovation.z() = wrapAngle(innovation.z());
    stats_.last_innovation_z_m = std::abs(innovation.x());
    stats_.last_innovation_tilt_deg =
        innovation.tail<2>().norm() * kRadiansToDegrees;
    const Eigen::Matrix3d innovation_covariance =
        vertical_covariance_ + measurement_covariance;
    const Eigen::LDLT<Eigen::Matrix3d> decomposition(innovation_covariance);
    if (decomposition.info() == Eigen::Success) {
      mahalanobis = innovation.dot(decomposition.solve(innovation));
    }
    const bool rebootstrap =
        !have_rail_vertical_fix_ ||
        raw.odom_distance_m - last_rail_fix_distance_m_ >=
            config_.rail_rebootstrap_gap_m;
    if (rebootstrap) {
      // The first valid rail fit establishes height/tilt at its local anchor.
      // Do not compare it against the rigid map seed: that seed has no model
      // for the height drift accumulated before rail coverage begins.
      const double updated_roll = raw.constrain_roll ? measurement.y()
                                                     : current_rpy.x();
      const double updated_pitch = raw.constrain_pitch ? measurement.z()
                                                       : current_rpy.y();
      const Rotation updated_rotation =
          rpyRotation(updated_roll, updated_pitch, current_rpy.z());
      const V3 rotated_anchor = updated_rotation * raw.correction_anchor_odom;
      // Tilt is observed at this rail window, not at the odom origin. Keep the
      // current horizontal anchor fixed here so vertical rebootstrap cannot
      // leak the rail's ambiguous cross-track/yaw solution into x/y before a
      // pole fix has identified the correct track. Once initialized, the
      // graph's RailPoseFactor applies the gated cross-track constraint.
      state_.x() = filtered_anchor_map.x() - rotated_anchor.x();
      state_.y() = filtered_anchor_map.y() - rotated_anchor.y();
      vertical_state_.x() = raw_anchor_map.z() - rotated_anchor.z();
      vertical_state_.y() = updated_roll;
      vertical_state_.z() = updated_pitch;
      vertical_covariance_ = measurement_covariance;
      have_rail_vertical_fix_ = true;
      last_rail_fix_distance_m_ = raw.odom_distance_m;
      accepted = true;
      ++stats_.rail_accepted;
      const bool allow_rail_graph =
          status_ == Status::Tracking || status_ == Status::Degraded ||
          status_ == Status::Lost;
      if (graph_ && allow_rail_graph) {
        const double roll_sigma = raw.constrain_roll ? tilt_sigma : 1.0e6;
        const double pitch_sigma = raw.constrain_pitch ? tilt_sigma : 1.0e6;
        const double graph_z_sigma = raw.constrain_z ? z_sigma : 1.0e6;
        const double graph_cross_sigma =
            raw.constrain_cross_track ? cross_track_sigma : 1.0e6;
        const double graph_yaw_sigma =
            raw.constrain_yaw ? rail_yaw_sigma : 1.0e6;
        const bool added = graph_->addRailMeasurement(
            raw.stamp, raw.T_map_odom, raw.rail_tangent_map,
            graph_cross_sigma, graph_z_sigma, roll_sigma, pitch_sigma,
            graph_yaw_sigma);
        if (added) {
          if (const auto optimized = graph_->correctionAt(raw.stamp)) {
            syncStateFromCorrectionLocked(*optimized);
          }
        } else {
          LOG(W, "Could not attach rail factor to keyframe at " << raw.stamp
                                                                 << ".");
        }
      }
      filtered = correctionLocked();
      filtered_covariance = vertical_covariance_;
      planar_covariance = covariance_;
      const bool has_confirmed_fix = status_ == Status::Tracking ||
                                     status_ == Status::Degraded ||
                                     status_ == Status::Lost;
      if (has_confirmed_fix) {
        if (graph_ && graph_->initialized()) rewriteTrajectoryFromGraphLocked();
        else rewriteTrajectoryTailLocked(raw.stamp, filtered);
      }
    } else if (stats_.last_innovation_z_m <= config_.max_innovation_z_m &&
        stats_.last_innovation_tilt_deg <= config_.max_innovation_tilt_deg &&
        mahalanobis <= config_.rail_mahalanobis_gate) {
      if (config_.filter_enable) {
        const Eigen::Matrix3d gain =
            vertical_covariance_ * decomposition.solve(Eigen::Matrix3d::Identity());
        const Eigen::Vector3d update = gain * innovation;
        const double updated_roll = wrapAngle(current_rpy.x() + update.y());
        const double updated_pitch = wrapAngle(current_rpy.y() + update.z());
        const double updated_anchor_z = filtered_anchor_map.z() + update.x();
        const Rotation updated_rotation =
            rpyRotation(updated_roll, updated_pitch, current_rpy.z());
        const V3 rotated_anchor = updated_rotation * raw.correction_anchor_odom;
        state_.x() = filtered_anchor_map.x() - rotated_anchor.x();
        state_.y() = filtered_anchor_map.y() - rotated_anchor.y();
        vertical_state_.x() = updated_anchor_z - rotated_anchor.z();
        vertical_state_.y() = updated_roll;
        vertical_state_.z() = updated_pitch;
        const Eigen::Matrix3d residual = Eigen::Matrix3d::Identity() - gain;
        vertical_covariance_ =
            residual * vertical_covariance_ * residual.transpose() +
            gain * measurement_covariance * gain.transpose();
      } else {
        vertical_state_ = measurement;
        vertical_covariance_ = measurement_covariance;
      }
      accepted = true;
      ++stats_.rail_accepted;
      last_rail_fix_distance_m_ = raw.odom_distance_m;
      const bool allow_rail_graph =
          status_ == Status::Tracking || status_ == Status::Degraded ||
          status_ == Status::Lost;
      if (graph_ && allow_rail_graph) {
        const double roll_sigma = raw.constrain_roll ? tilt_sigma : 1.0e6;
        const double pitch_sigma = raw.constrain_pitch ? tilt_sigma : 1.0e6;
        const double graph_z_sigma = raw.constrain_z ? z_sigma : 1.0e6;
        const double graph_cross_sigma =
            raw.constrain_cross_track ? cross_track_sigma : 1.0e6;
        const double graph_yaw_sigma =
            raw.constrain_yaw ? rail_yaw_sigma : 1.0e6;
        const bool added = graph_->addRailMeasurement(
            raw.stamp, raw.T_map_odom, raw.rail_tangent_map,
            graph_cross_sigma, graph_z_sigma, roll_sigma, pitch_sigma,
            graph_yaw_sigma);
        if (added) {
          if (const auto optimized = graph_->correctionAt(raw.stamp)) {
            syncStateFromCorrectionLocked(*optimized);
          }
        } else {
          LOG(W, "Could not attach rail factor to keyframe at " << raw.stamp
                                                                 << ".");
        }
      }
      filtered = correctionLocked();
      filtered_covariance = vertical_covariance_;
      planar_covariance = covariance_;
      const bool has_confirmed_fix = status_ == Status::Tracking ||
                                     status_ == Status::Degraded ||
                                     status_ == Status::Lost;
      if (has_confirmed_fix) {
        if (graph_ && graph_->initialized()) rewriteTrajectoryFromGraphLocked();
        else rewriteTrajectoryTailLocked(raw.stamp, filtered);
      }
    } else {
      ++stats_.rail_rejected;
      LOG(W, "Rejected filtered rail update: innovation "
                 << stats_.last_innovation_z_m << " m / "
                 << stats_.last_innovation_tilt_deg << " deg, Mahalanobis "
                 << mahalanobis << ".");
    }
  }
  if (!accepted) return;
  nav_msgs::Odometry message;
  message.header.stamp.fromNSec(raw.stamp);
  message.header.frame_id = config_.map_frame;
  message.child_frame_id = odom_frame_;
  transformToMsg(filtered, message.pose.pose);
  message.pose.covariance[0] = planar_covariance(0, 0);
  message.pose.covariance[7] = planar_covariance(1, 1);
  message.pose.covariance[14] = filtered_covariance(0, 0);
  message.pose.covariance[21] = filtered_covariance(1, 1);
  message.pose.covariance[28] = filtered_covariance(2, 2);
  message.pose.covariance[35] = planar_covariance(2, 2);
  correction_pub_.publish(message);
}

void PoleLocalization::addOdometry(uint64_t stamp, const Transform& T_odom_body) {
  Status old_status = Status::Uninitialized;
  Status new_status = Status::Uninitialized;
  bool publish_pose = false;
  bool publish_path = false;
  Transform correction;
  Transform T_map_body;
  Eigen::Matrix3d pose_covariance = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d vertical_pose_covariance = Eigen::Matrix3d::Identity();
  Path path_snapshot;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    old_status = status_;
    double traveled = 0.0;
    if (have_odom_) {
      traveled = (T_odom_body.translation() - last_odom_position_).head<2>().norm();
    }
    have_odom_ = true;
    latest_odom_stamp_ = stamp;
    latest_odom_ = T_odom_body;
    last_odom_position_ = T_odom_body.translation();
    odom_distance_m_ += traveled;

    if (initialized_ && have_accepted_measurement_distance_) {
      distance_since_measurement_ =
          std::max(0.0, odom_distance_m_ - last_accepted_measurement_distance_m_);
      if (status_ == Status::Tracking &&
          distance_since_measurement_ >= config_.degraded_after_distance_m) {
        status_ = Status::Degraded;
        confirmations_ = 0;
      }
      if ((status_ == Status::Tracking || status_ == Status::Degraded) &&
          distance_since_measurement_ >= config_.lost_after_distance_m) {
        status_ = Status::Lost;
        confirmations_ = 0;
      }
    }
    new_status = status_;

    const bool has_confirmed_fix = status_ == Status::Tracking ||
                                   status_ == Status::Degraded || status_ == Status::Lost;
    if (!has_confirmed_fix) {
      prelocalization_buffer_.push_back({stamp, T_odom_body});
      if (prelocalization_buffer_.size() > config_.prelocalization_buffer_frames) {
        prelocalization_buffer_.erase(
            prelocalization_buffer_.begin(),
            prelocalization_buffer_.begin() +
                static_cast<std::ptrdiff_t>(prelocalization_buffer_.size() -
                                            config_.prelocalization_buffer_frames));
      }
    }
    if (has_confirmed_fix && stamp != last_recorded_stamp_) {
      correction = correctionLocked();
      T_map_body = Transform(Eigen::Isometry3d(correction * T_odom_body));
      pose_covariance = covariance_;
      vertical_pose_covariance = vertical_covariance_;
      if (have_filter_distance_) {
        const double prediction_distance =
            std::max(0.0, odom_distance_m_ - filter_distance_m_);
        const double xy_process = config_.process_noise_xy_m_per_sqrt_m;
        const double yaw_process =
            config_.process_noise_yaw_deg_per_sqrt_m * kDegreesToRadians;
        pose_covariance(0, 0) += xy_process * xy_process * prediction_distance;
        pose_covariance(1, 1) += xy_process * xy_process * prediction_distance;
        pose_covariance(2, 2) += yaw_process * yaw_process * prediction_distance;
      }
      if (have_vertical_filter_distance_) {
        const double prediction_distance = std::max(
            0.0, odom_distance_m_ - vertical_filter_distance_m_);
        const double z_process = config_.process_noise_z_m_per_sqrt_m;
        const double tilt_process =
            config_.process_noise_tilt_deg_per_sqrt_m * kDegreesToRadians;
        vertical_pose_covariance(0, 0) +=
            z_process * z_process * prediction_distance;
        vertical_pose_covariance(1, 1) +=
            tilt_process * tilt_process * prediction_distance;
        vertical_pose_covariance(2, 2) +=
            tilt_process * tilt_process * prediction_distance;
      }
      trajectory_.poses.push_back({stamp, T_map_body});
      path_.poses.push_back({stamp, T_map_body});
      localized_odometry_.push_back({stamp, T_odom_body});
      if (config_.path_max_poses > 0 && path_.poses.size() > config_.path_max_poses) {
        path_.poses.erase(path_.poses.begin(),
                          path_.poses.begin() +
                              static_cast<std::ptrdiff_t>(path_.poses.size() -
                                                          config_.path_max_poses));
      }
      last_recorded_stamp_ = stamp;
      ++stats_.localized_frames;
      publish_pose = true;

      const uint64_t period = secondsToNanoseconds(1.0 / config_.path_publish_frequency);
      if (config_.publish_path &&
          (last_path_publish_stamp_ == 0 || stamp - last_path_publish_stamp_ >= period)) {
        last_path_publish_stamp_ = stamp;
        path_snapshot = path_;
        publish_path = true;
      }
    }
  }

  if (new_status != old_status) {
    publishStatus(new_status);
    LOG(I, "Pole localization status: " << toString(new_status) << ".");
  }
  if (!publish_pose) return;

  geometry_msgs::TransformStamped transform_message;
  transform_message.header.stamp.fromNSec(stamp);
  transform_message.header.frame_id = config_.map_frame;
  transform_message.child_frame_id = odom_frame_;
  transformToMsg(correction, transform_message.transform);
  tf_.sendTransform(transform_message);

  nav_msgs::Odometry odometry;
  odometry.header = transform_message.header;
  odometry.child_frame_id = body_frame_;
  transformToMsg(T_map_body, odometry.pose.pose);
  odometry.pose.covariance[0] = pose_covariance(0, 0);
  odometry.pose.covariance[7] = pose_covariance(1, 1);
  odometry.pose.covariance[14] = vertical_pose_covariance(0, 0);
  odometry.pose.covariance[21] = vertical_pose_covariance(1, 1);
  odometry.pose.covariance[28] = vertical_pose_covariance(2, 2);
  odometry.pose.covariance[35] = pose_covariance(2, 2);
  odom_pub_.publish(odometry);

  if (publish_path) {
    Header header;
    header.stamp = stamp;
    header.frame = config_.map_frame;
    nav_msgs::Path path_message;
    pathToMsg(path_snapshot, header, path_message);
    path_pub_.publish(path_message);
  }
}

bool PoleLocalization::saveResults(std::string* message) const {
  namespace fs = std::filesystem;
  Path trajectory;
  Stats stats;
  Status status;
  Eigen::Matrix3d covariance;
  Eigen::Matrix3d vertical_covariance;
  std::vector<UpdateRecord> updates;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    trajectory = trajectory_;
    stats = stats_;
    status = status_;
    covariance = covariance_;
    vertical_covariance = vertical_covariance_;
    updates = updates_;
  }
  if (trajectory.poses.empty()) {
    if (message) *message = "No pole-localized poses accumulated; trajectory not saved.";
    return false;
  }

  std::error_code error;
  fs::create_directories(config_.output_directory, error);
  if (error) {
    if (message) *message = "Failed to create pole localization output: " + error.message();
    return false;
  }
  const fs::path directory(config_.output_directory);
  std::ofstream tum(directory / "localization_tum.txt");
  std::ofstream summary(directory / "summary.yaml");
  std::ofstream corrections(directory / "filtered_corrections.txt");
  if (!tum || !summary || !corrections) {
    if (message) *message = "Failed to open pole localization result files.";
    return false;
  }

  tum << std::fixed << std::setprecision(9);
  for (const StampedPose& stamped : trajectory.poses) {
    const V3& position = stamped.pose.translation();
    const Quaternion orientation(stamped.pose.rotation());
    tum << nsToS(stamped.stamp) << " " << position.x() << " " << position.y() << " "
        << position.z() << " " << orientation.x() << " " << orientation.y() << " "
        << orientation.z() << " " << orientation.w() << "\n";
  }
  summary << "status: " << toString(status) << "\n"
          << "measurements: " << stats.measurements << "\n"
          << "accepted: " << stats.accepted << "\n"
          << "rejected: " << stats.rejected << "\n"
          << "localized_frames: " << stats.localized_frames << "\n"
          << "rail_measurements: " << stats.rail_measurements << "\n"
          << "rail_accepted: " << stats.rail_accepted << "\n"
          << "rail_rejected: " << stats.rail_rejected << "\n"
          << "last_innovation_m: " << stats.last_innovation_m << "\n"
          << "last_innovation_yaw_deg: " << stats.last_innovation_yaw_deg << "\n"
          << "last_innovation_z_m: " << stats.last_innovation_z_m << "\n"
          << "last_innovation_tilt_deg: " << stats.last_innovation_tilt_deg << "\n"
          << "covariance_x: " << covariance(0, 0) << "\n"
          << "covariance_y: " << covariance(1, 1) << "\n"
          << "covariance_yaw: " << covariance(2, 2) << "\n"
          << "covariance_z: " << vertical_covariance(0, 0) << "\n"
          << "covariance_roll: " << vertical_covariance(1, 1) << "\n"
          << "covariance_pitch: " << vertical_covariance(2, 2) << "\n";
  corrections
      << "Timestamp Accepted Status RawX RawY RawYawDeg FilteredX FilteredY "
         "FilteredYawDeg Matches MatchRatio Baseline RMS InnovationM "
         "InnovationYawDeg Mahalanobis CovX CovY CovYaw\n"
      << std::fixed << std::setprecision(9);
  for (const UpdateRecord& update : updates) {
    corrections << nsToS(update.stamp) << " " << (update.accepted ? 1 : 0) << " "
                << toString(update.status) << " " << update.raw.translation().x() << " "
                << update.raw.translation().y() << " "
                << yawOf(update.raw.rotation()) * kRadiansToDegrees << " "
                << update.filtered.translation().x() << " "
                << update.filtered.translation().y() << " "
                << yawOf(update.filtered.rotation()) * kRadiansToDegrees << " "
                << update.matches << " " << update.match_ratio << " " << update.baseline
                << " " << update.rms << " " << update.innovation_m << " "
                << update.innovation_yaw_deg << " " << update.mahalanobis << " "
                << update.covariance(0, 0) << " " << update.covariance(1, 1) << " "
                << update.covariance(2, 2) << "\n";
  }
  if (!tum || !summary || !corrections) {
    if (message) *message = "Failed while writing pole localization results.";
    return false;
  }
  if (message) {
    *message = "Saved " + std::to_string(trajectory.poses.size()) +
               " pole-localized poses to '" + directory.string() + "'.";
  }
  return true;
}

}  // namespace bievr
