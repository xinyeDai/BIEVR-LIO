#include "bievr_pgo/gnss_optimizer.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace bievr {
namespace {

constexpr double kPi = 3.14159265358979323846;

gtsam::Pose3 toPose3(const Transform& pose) {
  return gtsam::Pose3(gtsam::Rot3(pose.rotation()), gtsam::Point3(pose.translation()));
}

Transform toTransform(const gtsam::Pose3& pose) {
  return Transform(pose.rotation().matrix(), pose.translation());
}

// Position observation at a timestamp bracketed by two LIO poses. This avoids
// a speed-dependent error (up to half a LiDAR period) from nearest-neighbour
// timestamp association.
class InterpolatedPositionFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;

  InterpolatedPositionFactor(gtsam::Key first, gtsam::Key second, double alpha,
                             const gtsam::Point3& measurement,
                             const gtsam::SharedNoiseModel& model)
      : Base(model, first, second), alpha_(alpha), measurement_(measurement) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& first, const gtsam::Pose3& second,
      boost::optional<gtsam::Matrix&> H_first = boost::none,
      boost::optional<gtsam::Matrix&> H_second = boost::none) const override {
    gtsam::Matrix36 J_first;
    gtsam::Matrix36 J_second;
    const gtsam::Point3 p_first = first.translation(H_first ? &J_first : nullptr);
    const gtsam::Point3 p_second = second.translation(H_second ? &J_second : nullptr);
    if (H_first) *H_first = (1.0 - alpha_) * J_first;
    if (H_second) *H_second = alpha_ * J_second;
    return (1.0 - alpha_) * p_first + alpha_ * p_second - measurement_;
  }

 private:
  double alpha_;
  gtsam::Point3 measurement_;
};

struct Association {
  size_t first = 0;
  size_t second = 0;
  double alpha = 0.0;
  const GnssTrajectoryOptimizer::PositionMeasurement* measurement = nullptr;
};

V3 interpolatePosition(const std::vector<GnssTrajectoryOptimizer::StampedPose>& poses,
                       const Association& association) {
  return (1.0 - association.alpha) * poses[association.first].pose.translation() +
         association.alpha * poses[association.second].pose.translation();
}

bool associate(const std::vector<GnssTrajectoryOptimizer::StampedPose>& poses,
               const GnssTrajectoryOptimizer::PositionMeasurement& measurement,
               double max_gap_s, Association& output) {
  const auto upper = std::lower_bound(
      poses.begin(), poses.end(), measurement.stamp,
      [](const GnssTrajectoryOptimizer::StampedPose& pose, uint64_t stamp) {
        return pose.stamp < stamp;
      });
  if (upper == poses.begin() || upper == poses.end()) return false;
  output.second = static_cast<size_t>(upper - poses.begin());
  output.first = output.second - 1;
  const uint64_t interval = poses[output.second].stamp - poses[output.first].stamp;
  if (interval == 0) return false;
  const double before_s = static_cast<double>(measurement.stamp - poses[output.first].stamp) * 1e-9;
  const double after_s = static_cast<double>(poses[output.second].stamp - measurement.stamp) * 1e-9;
  if (std::min(before_s, after_s) > max_gap_s ||
      static_cast<double>(interval) * 1e-9 > 2.0 * max_gap_s) return false;
  output.alpha = static_cast<double>(measurement.stamp - poses[output.first].stamp) /
                 static_cast<double>(interval);
  output.measurement = &measurement;
  return true;
}

Transform planarAlignment(const std::vector<GnssTrajectoryOptimizer::StampedPose>& lio,
                          const std::vector<Association>& associations) {
  V3 source_mean = V3::Zero();
  V3 target_mean = V3::Zero();
  for (const Association& association : associations) {
    source_mean += interpolatePosition(lio, association);
    target_mean += association.measurement->position;
  }
  const double count = static_cast<double>(associations.size());
  source_mean /= count;
  target_mean /= count;
  double dot = 0.0;
  double cross = 0.0;
  for (const Association& association : associations) {
    const V3 source = interpolatePosition(lio, association) - source_mean;
    const V3 target = association.measurement->position - target_mean;
    dot += source.x() * target.x() + source.y() * target.y();
    cross += source.x() * target.y() - source.y() * target.x();
  }
  const double yaw = std::atan2(cross, dot);
  const Rotation rotation = Eigen::AngleAxisd(yaw, V3::UnitZ()).toRotationMatrix();
  return Transform(rotation, target_mean - rotation * source_mean);
}

struct ResidualStats {
  double horizontal_rmse = 0.0;
  double three_dimensional_rmse = 0.0;
};

ResidualStats residualStats(
    const std::vector<GnssTrajectoryOptimizer::StampedPose>& poses,
    const std::vector<Association>& associations) {
  double horizontal_squared = 0.0;
  double three_dimensional_squared = 0.0;
  for (const Association& association : associations) {
    const V3 residual = interpolatePosition(poses, association) -
                        association.measurement->position;
    horizontal_squared += residual.head<2>().squaredNorm();
    three_dimensional_squared += residual.squaredNorm();
  }
  ResidualStats result;
  if (!associations.empty()) {
    result.horizontal_rmse =
        std::sqrt(horizontal_squared / static_cast<double>(associations.size()));
    result.three_dimensional_rmse =
        std::sqrt(three_dimensional_squared / static_cast<double>(associations.size()));
  }
  return result;
}

}  // namespace

GnssTrajectoryOptimizer::GnssTrajectoryOptimizer() = default;

GnssTrajectoryOptimizer::GnssTrajectoryOptimizer(Config config)
    : config_(std::move(config)) {}

bool GnssTrajectoryOptimizer::optimize(
    const std::vector<StampedPose>& lio,
    const std::vector<PositionMeasurement>& gnss,
    std::vector<StampedPose>& optimized, Stats& stats, std::string* message) const {
  optimized.clear();
  stats = Stats();
  stats.input_poses = lio.size();
  stats.input_measurements = gnss.size();
  if (lio.size() < 2 || gnss.size() < 3) {
    if (message) *message = "GNSS optimization needs at least two LIO poses and three fixes.";
    return false;
  }
  if (config_.max_time_gap_s <= 0.0 || config_.horizontal_sigma_floor_m <= 0.0 ||
      config_.vertical_sigma_floor_m <= 0.0 || config_.gnss_huber_delta <= 0.0 ||
      config_.initial_outlier_gate_m <= 0.0 ||
      config_.odom_rotation_sigma_deg_per_sqrt_m <= 0.0 ||
      config_.odom_translation_sigma_m_per_sqrt_m <= 0.0 ||
      config_.minimum_edge_distance_m <= 0.0 || config_.max_iterations <= 0) {
    if (message) *message = "Invalid non-positive GNSS optimizer configuration.";
    return false;
  }
  if (!std::is_sorted(lio.begin(), lio.end(),
                      [](const StampedPose& a, const StampedPose& b) {
                        return a.stamp < b.stamp;
                      })) {
    if (message) *message = "LIO trajectory timestamps must be strictly ordered.";
    return false;
  }

  std::vector<Association> candidates;
  candidates.reserve(gnss.size());
  for (const PositionMeasurement& measurement : gnss) {
    Association association;
    if (measurement.position.allFinite() && measurement.variance.allFinite() &&
        associate(lio, measurement, config_.max_time_gap_s, association)) {
      candidates.push_back(association);
    } else {
      ++stats.rejected_time;
    }
  }
  if (candidates.size() < 3) {
    if (message) *message = "Fewer than three GNSS fixes overlap the LIO trajectory.";
    return false;
  }

  stats.initial_T_enu_odom = planarAlignment(lio, candidates);
  std::vector<StampedPose> initialized = lio;
  for (StampedPose& pose : initialized) {
    pose.pose = Transform(stats.initial_T_enu_odom * pose.pose);
  }

  std::vector<Association> associations;
  associations.reserve(candidates.size());
  for (const Association& association : candidates) {
    const V3 residual = interpolatePosition(initialized, association) -
                        association.measurement->position;
    if (residual.head<2>().norm() <= config_.initial_outlier_gate_m) {
      associations.push_back(association);
    } else {
      ++stats.rejected_outlier;
    }
  }
  if (associations.size() < 3) {
    if (message) *message = "GNSS outlier gate retained fewer than three fixes.";
    return false;
  }
  stats.used_measurements = associations.size();
  const ResidualStats before = residualStats(initialized, associations);
  stats.initial_horizontal_rmse_m = before.horizontal_rmse;
  stats.initial_3d_rmse_m = before.three_dimensional_rmse;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  for (size_t index = 0; index < initialized.size(); ++index) {
    values.insert(index, toPose3(initialized[index].pose));
  }

  const double degrees_to_radians = kPi / 180.0;
  gtsam::Vector6 prior_sigmas;
  prior_sigmas << config_.prior_roll_pitch_sigma_deg * degrees_to_radians,
      config_.prior_roll_pitch_sigma_deg * degrees_to_radians,
      config_.prior_yaw_sigma_deg * degrees_to_radians,
      config_.prior_translation_sigma_m, config_.prior_translation_sigma_m,
      config_.prior_translation_sigma_m;
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      0, toPose3(initialized.front().pose),
      gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas)));

  for (size_t index = 1; index < lio.size(); ++index) {
    const gtsam::Pose3 previous = toPose3(lio[index - 1].pose);
    const gtsam::Pose3 current = toPose3(lio[index].pose);
    const double distance = std::max(
        config_.minimum_edge_distance_m,
        (lio[index].pose.translation() - lio[index - 1].pose.translation()).norm());
    const double scale = std::sqrt(distance);
    const double rotation_sigma =
        config_.odom_rotation_sigma_deg_per_sqrt_m * degrees_to_radians * scale;
    const double translation_sigma =
        config_.odom_translation_sigma_m_per_sqrt_m * scale;
    gtsam::Vector6 sigmas;
    sigmas << rotation_sigma, rotation_sigma, rotation_sigma, translation_sigma,
        translation_sigma, translation_sigma;
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        index - 1, index, previous.between(current),
        gtsam::noiseModel::Diagonal::Sigmas(sigmas)));
  }

  for (const Association& association : associations) {
    const V3 variance = association.measurement->variance.cwiseMax(V3::Zero());
    gtsam::Vector3 sigmas;
    sigmas << std::max(std::sqrt(variance.x()), config_.horizontal_sigma_floor_m),
        std::max(std::sqrt(variance.y()), config_.horizontal_sigma_floor_m),
        std::max(std::sqrt(variance.z()), config_.vertical_sigma_floor_m);
    const auto gaussian = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    const auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(config_.gnss_huber_delta), gaussian);
    graph.emplace_shared<InterpolatedPositionFactor>(
        association.first, association.second, association.alpha,
        gtsam::Point3(association.measurement->position), robust);
  }

  try {
    stats.initial_error = graph.error(values);
    gtsam::LevenbergMarquardtParams parameters;
    parameters.maxIterations = config_.max_iterations;
    parameters.setVerbosityLM("SILENT");
    const gtsam::Values result =
        gtsam::LevenbergMarquardtOptimizer(graph, values, parameters).optimize();
    stats.final_error = graph.error(result);
    optimized.reserve(lio.size());
    for (size_t index = 0; index < lio.size(); ++index) {
      optimized.push_back({lio[index].stamp, toTransform(result.at<gtsam::Pose3>(index))});
    }
  } catch (const std::exception& error) {
    if (message) *message = std::string("GTSAM optimization failed: ") + error.what();
    return false;
  }

  const ResidualStats after = residualStats(optimized, associations);
  stats.optimized_horizontal_rmse_m = after.horizontal_rmse;
  stats.optimized_3d_rmse_m = after.three_dimensional_rmse;
  if (message) {
    std::ostringstream stream;
    stream << "Optimized " << optimized.size() << " LIO poses with "
           << stats.used_measurements << " GNSS factors; horizontal RMSE "
           << stats.initial_horizontal_rmse_m << " -> "
           << stats.optimized_horizontal_rmse_m << " m.";
    *message = stream.str();
  }
  return true;
}

}  // namespace bievr
