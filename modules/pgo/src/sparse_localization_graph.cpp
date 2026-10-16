#include "bievr_pgo/sparse_localization_graph.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <utility>

namespace bievr {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegreesToRadians = kPi / 180.0;

double wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

gtsam::Pose3 toPose3(const Transform& pose) {
  return gtsam::Pose3(gtsam::Rot3(pose.rotation()),
                      gtsam::Point3(pose.translation()));
}

Transform toTransform(const gtsam::Pose3& pose) {
  return Transform(pose.rotation().matrix(), pose.translation());
}

Eigen::Vector3d rpyOf(const gtsam::Rot3& rotation) {
  const Eigen::Matrix3d matrix = rotation.matrix();
  const double pitch = std::asin(std::clamp(-matrix(2, 0), -1.0, 1.0));
  const double roll = std::atan2(matrix(2, 1), matrix(2, 2));
  const double yaw = std::atan2(matrix(1, 0), matrix(0, 0));
  return Eigen::Vector3d(roll, pitch, yaw);
}

template <typename ErrorFunction>
gtsam::Vector errorWithNumericalJacobian(const gtsam::Pose3& pose,
                                         ErrorFunction&& function,
                                         boost::optional<gtsam::Matrix&> H) {
  const gtsam::Vector error = function(pose);
  if (!H) return error;
  H->resize(error.size(), 6);
  constexpr double kStep = 1.0e-6;
  for (int column = 0; column < 6; ++column) {
    gtsam::Vector6 delta = gtsam::Vector6::Zero();
    delta[column] = kStep;
    H->col(column) = (function(pose.retract(delta)) - error) / kStep;
  }
  return error;
}

class PlanarPoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  PlanarPoseFactor(gtsam::Key key, const gtsam::Pose3& measurement,
                   const gtsam::SharedNoiseModel& model)
      : Base(model, key), measurement_(measurement),
        measurement_yaw_(rpyOf(measurement.rotation()).z()) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    const auto error = [this](const gtsam::Pose3& candidate) {
      gtsam::Vector3 output;
      output << candidate.translation().x() - measurement_.translation().x(),
          candidate.translation().y() - measurement_.translation().y(),
          wrapAngle(rpyOf(candidate.rotation()).z() - measurement_yaw_);
      return output;
    };
    return errorWithNumericalJacobian(pose, error, H);
  }

 private:
  gtsam::Pose3 measurement_;
  double measurement_yaw_ = 0.0;
};

class VerticalPoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  VerticalPoseFactor(gtsam::Key key, const gtsam::Pose3& measurement,
                     const gtsam::SharedNoiseModel& model)
      : Base(model, key), measurement_(measurement),
        measurement_rpy_(rpyOf(measurement.rotation())) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    const auto error = [this](const gtsam::Pose3& candidate) {
      const Eigen::Vector3d rpy = rpyOf(candidate.rotation());
      gtsam::Vector3 output;
      output << candidate.translation().z() - measurement_.translation().z(),
          wrapAngle(rpy.x() - measurement_rpy_.x()),
          wrapAngle(rpy.y() - measurement_rpy_.y());
      return output;
    };
    return errorWithNumericalJacobian(pose, error, H);
  }

 private:
  gtsam::Pose3 measurement_;
  Eigen::Vector3d measurement_rpy_ = Eigen::Vector3d::Zero();
};

class RailPoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor1<gtsam::Pose3>;

  RailPoseFactor(gtsam::Key key, const gtsam::Pose3& measurement,
                 const Eigen::Vector2d& tangent_map,
                 const gtsam::SharedNoiseModel& model)
      : Base(model, key), measurement_(measurement),
        measurement_rpy_(rpyOf(measurement.rotation())) {
    Eigen::Vector2d tangent = tangent_map;
    if (tangent.norm() < 1.0e-8) tangent = Eigen::Vector2d::UnitX();
    tangent.normalize();
    normal_map_ = Eigen::Vector2d(-tangent.y(), tangent.x());
  }

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    const auto error = [this](const gtsam::Pose3& candidate) {
      const Eigen::Vector3d rpy = rpyOf(candidate.rotation());
      gtsam::Vector output(5);
      const Eigen::Vector2d translation_error(
          candidate.translation().x() - measurement_.translation().x(),
          candidate.translation().y() - measurement_.translation().y());
      output << normal_map_.dot(translation_error),
          candidate.translation().z() - measurement_.translation().z(),
          wrapAngle(rpy.x() - measurement_rpy_.x()),
          wrapAngle(rpy.y() - measurement_rpy_.y()),
          wrapAngle(rpy.z() - measurement_rpy_.z());
      return output;
    };
    return errorWithNumericalJacobian(pose, error, H);
  }

 private:
  gtsam::Pose3 measurement_;
  Eigen::Vector3d measurement_rpy_ = Eigen::Vector3d::Zero();
  Eigen::Vector2d normal_map_ = Eigen::Vector2d::UnitY();
};

template <typename Derived>
gtsam::SharedNoiseModel robustNoise(const Eigen::MatrixBase<Derived>& sigmas,
                                    double huber_delta) {
  const gtsam::Vector dynamic_sigmas = sigmas;
  const auto diagonal = gtsam::noiseModel::Diagonal::Sigmas(dynamic_sigmas);
  if (huber_delta <= 0.0) return diagonal;
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(huber_delta), diagonal);
}

Transform interpolateTransform(const Transform& first, const Transform& second,
                               double alpha) {
  const Quaternion rotation = Quaternion(first.rotation())
                                  .slerp(alpha, Quaternion(second.rotation()))
                                  .normalized();
  return Transform(rotation, (1.0 - alpha) * first.translation() +
                                 alpha * second.translation());
}

}  // namespace

class SparseLocalizationGraph::Impl {
 public:
  explicit Impl(Config config) : config_(std::move(config)) { resetIsam(); }

  void resetIsam() {
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = config_.isam_relinearize_threshold;
    params.relinearizeSkip = config_.isam_relinearize_skip;
    isam_ = std::make_unique<gtsam::ISAM2>(params);
    estimate_.clear();
    initialized_ = false;
  }

  gtsam::Key key(size_t index) const { return gtsam::Symbol('x', index); }

  gtsam::SharedNoiseModel betweenNoise(const Transform& relative) const {
    const double distance = std::max(config_.minimum_edge_distance_m,
                                     relative.translation().norm());
    const double scale = std::sqrt(distance);
    gtsam::Vector6 sigmas;
    sigmas << config_.odom_tilt_sigma_deg_per_sqrt_m * kDegreesToRadians * scale,
        config_.odom_tilt_sigma_deg_per_sqrt_m * kDegreesToRadians * scale,
        config_.odom_yaw_sigma_deg_per_sqrt_m * kDegreesToRadians * scale,
        config_.odom_xy_sigma_m_per_sqrt_m * scale,
        config_.odom_xy_sigma_m_per_sqrt_m * scale,
        config_.odom_z_sigma_m_per_sqrt_m * scale;
    return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
  }

  void appendKeyframeToGraph(size_t index, const Transform& initial_map_pose) {
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    values.insert(key(index), toPose3(initial_map_pose));
    if (index == 0) {
      gtsam::Vector6 sigmas;
      const double rotation_sigma =
          config_.prior_rotation_sigma_deg * kDegreesToRadians;
      sigmas << rotation_sigma, rotation_sigma, rotation_sigma,
          config_.prior_translation_sigma_m, config_.prior_translation_sigma_m,
          config_.prior_translation_sigma_m;
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
          key(index), toPose3(initial_map_pose),
          gtsam::noiseModel::Diagonal::Sigmas(sigmas));
    } else {
      const Transform relative(Eigen::Isometry3d(
          keyframes_[index - 1].odom_pose.inverse() * keyframes_[index].odom_pose));
      factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          key(index - 1), key(index), toPose3(relative), betweenNoise(relative));
    }
    isam_->update(factors, values);
    estimate_ = isam_->calculateEstimate();
  }

  void initialize(const Transform& correction) {
    resetIsam();
    initial_correction_ = correction;
    for (size_t index = 0; index < keyframes_.size(); ++index) {
      const Transform initial(Eigen::Isometry3d(correction * keyframes_[index].odom_pose));
      appendKeyframeToGraph(index, initial);
    }
    initialized_ = true;
  }

  bool addKeyframe(uint64_t stamp, const Transform& odom_pose) {
    if (!keyframes_.empty() && stamp <= keyframes_.back().stamp) {
      return stamp == keyframes_.back().stamp;
    }
    const size_t index = keyframes_.size();
    keyframes_.push_back({stamp, odom_pose, Transform::Identity()});
    stamp_to_index_.emplace(stamp, index);
    if (!initialized_) return true;

    Transform initial;
    if (index == 0) {
      initial = Transform(Eigen::Isometry3d(initial_correction_ * odom_pose));
    } else {
      const gtsam::Pose3 previous = estimate_.at<gtsam::Pose3>(key(index - 1));
      const Transform relative(Eigen::Isometry3d(
          keyframes_[index - 1].odom_pose.inverse() * odom_pose));
      initial = Transform(Eigen::Isometry3d(toTransform(previous) * relative));
    }
    appendKeyframeToGraph(index, initial);
    return true;
  }

  bool measurementIndex(uint64_t stamp, size_t& index) const {
    const auto found = stamp_to_index_.find(stamp);
    if (found == stamp_to_index_.end()) return false;
    index = found->second;
    return true;
  }

  bool addPlanar(uint64_t stamp, const Transform& correction,
                 double xy_sigma, double yaw_sigma) {
    size_t index = 0;
    if (!measurementIndex(stamp, index)) return false;
    if (!initialized_) initialize(correction);
    const Transform measured(Eigen::Isometry3d(correction * keyframes_[index].odom_pose));
    gtsam::Vector3 sigmas;
    sigmas << xy_sigma, xy_sigma, yaw_sigma;
    gtsam::NonlinearFactorGraph factors;
    factors.emplace_shared<PlanarPoseFactor>(
        key(index), toPose3(measured), robustNoise(sigmas, config_.robust_huber_delta));
    isam_->update(factors, gtsam::Values());
    isam_->update();
    estimate_ = isam_->calculateEstimate();
    return true;
  }

  bool addVertical(uint64_t stamp, const Transform& correction, double z_sigma,
                   double roll_sigma, double pitch_sigma) {
    size_t index = 0;
    if (!initialized_ || !measurementIndex(stamp, index)) return false;
    const Transform measured(Eigen::Isometry3d(correction * keyframes_[index].odom_pose));
    gtsam::Vector3 sigmas;
    sigmas << z_sigma, roll_sigma, pitch_sigma;
    gtsam::NonlinearFactorGraph factors;
    factors.emplace_shared<VerticalPoseFactor>(
        key(index), toPose3(measured), robustNoise(sigmas, config_.robust_huber_delta));
    isam_->update(factors, gtsam::Values());
    isam_->update();
    estimate_ = isam_->calculateEstimate();
    return true;
  }

  bool addRail(uint64_t stamp, const Transform& correction,
               const V3& tangent_map, double cross_track_sigma,
               double z_sigma, double roll_sigma, double pitch_sigma,
               double yaw_sigma) {
    size_t index = 0;
    if (!initialized_ || !measurementIndex(stamp, index)) return false;
    const Transform measured(Eigen::Isometry3d(correction * keyframes_[index].odom_pose));
    gtsam::Vector sigmas(5);
    sigmas << cross_track_sigma, z_sigma, roll_sigma, pitch_sigma, yaw_sigma;
    gtsam::NonlinearFactorGraph factors;
    factors.emplace_shared<RailPoseFactor>(
        key(index), toPose3(measured), tangent_map.head<2>(),
        robustNoise(sigmas, config_.robust_huber_delta));
    isam_->update(factors, gtsam::Values());
    isam_->update();
    estimate_ = isam_->calculateEstimate();
    return true;
  }

  std::vector<KeyframeEstimate> estimates() const {
    std::vector<KeyframeEstimate> result;
    if (!initialized_) return result;
    result.reserve(keyframes_.size());
    for (size_t index = 0; index < keyframes_.size(); ++index) {
      result.push_back({keyframes_[index].stamp, keyframes_[index].odom_pose,
                        toTransform(estimate_.at<gtsam::Pose3>(key(index)))});
    }
    return result;
  }

  std::optional<Transform> correctionAt(uint64_t stamp) const {
    if (!initialized_ || keyframes_.empty()) return std::nullopt;
    auto correction = [this](size_t index) {
      return Transform(Eigen::Isometry3d(
          toTransform(estimate_.at<gtsam::Pose3>(key(index))) *
          keyframes_[index].odom_pose.inverse()));
    };
    const auto upper = std::lower_bound(
        keyframes_.begin(), keyframes_.end(), stamp,
        [](const KeyframeEstimate& frame, uint64_t value) {
          return frame.stamp < value;
        });
    if (upper == keyframes_.begin()) return correction(0);
    if (upper == keyframes_.end()) return correction(keyframes_.size() - 1);
    const size_t second = static_cast<size_t>(upper - keyframes_.begin());
    const size_t first = second - 1;
    const uint64_t duration = keyframes_[second].stamp - keyframes_[first].stamp;
    const double alpha = duration == 0
                             ? 0.0
                             : static_cast<double>(stamp - keyframes_[first].stamp) /
                                   static_cast<double>(duration);
    return interpolateTransform(correction(first), correction(second), alpha);
  }

  Config config_;
  bool initialized_ = false;
  Transform initial_correction_ = Transform::Identity();
  std::vector<KeyframeEstimate> keyframes_;
  std::unordered_map<uint64_t, size_t> stamp_to_index_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values estimate_;
};

SparseLocalizationGraph::SparseLocalizationGraph()
    : SparseLocalizationGraph(Config{}) {}

SparseLocalizationGraph::SparseLocalizationGraph(Config config)
    : impl_(new Impl(std::move(config))) {}

SparseLocalizationGraph::~SparseLocalizationGraph() = default;

void SparseLocalizationGraph::setInitialCorrection(const Transform& correction) {
  impl_->initialize(correction);
}

bool SparseLocalizationGraph::addKeyframe(uint64_t stamp,
                                          const Transform& odom_pose) {
  return impl_->addKeyframe(stamp, odom_pose);
}

bool SparseLocalizationGraph::addPlanarMeasurement(
    uint64_t stamp, const Transform& correction, double xy_sigma,
    double yaw_sigma) {
  return impl_->addPlanar(stamp, correction, xy_sigma, yaw_sigma);
}

bool SparseLocalizationGraph::addVerticalMeasurement(
    uint64_t stamp, const Transform& correction, double z_sigma,
    double roll_sigma, double pitch_sigma) {
  return impl_->addVertical(stamp, correction, z_sigma, roll_sigma, pitch_sigma);
}

bool SparseLocalizationGraph::addRailMeasurement(
    uint64_t stamp, const Transform& correction, const V3& tangent_map,
    double cross_track_sigma_m, double z_sigma_m, double roll_sigma_rad,
    double pitch_sigma_rad, double yaw_sigma_rad) {
  return impl_->addRail(stamp, correction, tangent_map, cross_track_sigma_m,
                        z_sigma_m, roll_sigma_rad, pitch_sigma_rad,
                        yaw_sigma_rad);
}

bool SparseLocalizationGraph::initialized() const { return impl_->initialized_; }

size_t SparseLocalizationGraph::size() const { return impl_->keyframes_.size(); }

std::optional<Transform> SparseLocalizationGraph::correctionAt(uint64_t stamp) const {
  return impl_->correctionAt(stamp);
}

std::vector<SparseLocalizationGraph::KeyframeEstimate>
SparseLocalizationGraph::estimates() const {
  return impl_->estimates();
}

}  // namespace bievr
