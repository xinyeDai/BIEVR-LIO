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
#include <cctype>
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
                             const gtsam::Point3& lever_arm,
                             const gtsam::SharedNoiseModel& model)
      : Base(model, first, second),
        alpha_(alpha),
        measurement_(measurement),
        lever_arm_(lever_arm) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& first, const gtsam::Pose3& second,
      boost::optional<gtsam::Matrix&> H_first = boost::none,
      boost::optional<gtsam::Matrix&> H_second = boost::none) const override {
    gtsam::Matrix36 J_first;
    gtsam::Matrix36 J_second;
    const gtsam::Point3 p_first =
        first.transformFrom(lever_arm_, H_first ? &J_first : nullptr);
    const gtsam::Point3 p_second =
        second.transformFrom(lever_arm_, H_second ? &J_second : nullptr);
    if (H_first) *H_first = (1.0 - alpha_) * J_first;
    if (H_second) *H_second = alpha_ * J_second;
    return (1.0 - alpha_) * p_first + alpha_ * p_second - measurement_;
  }

 private:
  double alpha_;
  gtsam::Point3 measurement_;
  gtsam::Point3 lever_arm_;
};

struct Association {
  size_t first = 0;
  size_t second = 0;
  double alpha = 0.0;
  const GnssTrajectoryOptimizer::PositionMeasurement* measurement = nullptr;
};

V3 antennaPosition(const GnssTrajectoryOptimizer::StampedPose& pose,
                   const V3& lever_arm) {
  return pose.pose.translation() + pose.pose.rotation() * lever_arm;
}

V3 interpolatePosition(const std::vector<GnssTrajectoryOptimizer::StampedPose>& poses,
                       const Association& association, const V3& lever_arm) {
  return (1.0 - association.alpha) *
             antennaPosition(poses[association.first], lever_arm) +
         association.alpha * antennaPosition(poses[association.second], lever_arm);
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

double median(std::vector<double> values) {
  const size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  const double upper = values[middle];
  if (values.size() % 2 != 0) return upper;
  std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
  return 0.5 * (values[middle - 1] + upper);
}

Transform planarAlignment(const std::vector<GnssTrajectoryOptimizer::StampedPose>& lio,
                          const std::vector<Association>& associations,
                          const V3& lever_arm) {
  V3 source_mean = V3::Zero();
  V3 target_mean = V3::Zero();
  for (const Association& association : associations) {
    source_mean += interpolatePosition(lio, association, lever_arm);
    target_mean += association.measurement->position;
  }
  const double count = static_cast<double>(associations.size());
  source_mean /= count;
  target_mean /= count;
  double dot = 0.0;
  double cross = 0.0;
  for (const Association& association : associations) {
    const V3 source = interpolatePosition(lio, association, lever_arm) - source_mean;
    const V3 target = association.measurement->position - target_mean;
    dot += source.x() * target.x() + source.y() * target.y();
    cross += source.x() * target.y() - source.y() * target.x();
  }
  const double yaw = std::atan2(cross, dot);
  const Rotation rotation = Eigen::AngleAxisd(yaw, V3::UnitZ()).toRotationMatrix();
  V3 translation = target_mean - rotation * source_mean;
  std::vector<double> height_offsets;
  height_offsets.reserve(associations.size());
  for (const Association& association : associations) {
    height_offsets.push_back(association.measurement->position.z() -
                             interpolatePosition(lio, association, lever_arm).z());
  }
  translation.z() = median(std::move(height_offsets));
  return Transform(rotation, translation);
}

struct ResidualStats {
  double horizontal_rmse = 0.0;
  double three_dimensional_rmse = 0.0;
};

ResidualStats residualStats(
    const std::vector<GnssTrajectoryOptimizer::StampedPose>& poses,
    const std::vector<Association>& associations, const V3& lever_arm) {
  double horizontal_squared = 0.0;
  double three_dimensional_squared = 0.0;
  for (const Association& association : associations) {
    const V3 residual = interpolatePosition(poses, association, lever_arm) -
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

gtsam::Vector3 gnssSigmas(
    const GnssTrajectoryOptimizer::PositionMeasurement& measurement,
    const GnssTrajectoryOptimizer::Config& config) {
  const V3 variance =
      (measurement.variance * config.gnss_covariance_scale).cwiseMax(V3::Zero());
  gtsam::Vector3 sigmas;
  sigmas << std::max(std::sqrt(variance.x()), config.horizontal_sigma_floor_m),
      std::max(std::sqrt(variance.y()), config.horizontal_sigma_floor_m),
      std::max(std::sqrt(variance.z()), config.vertical_sigma_floor_m);
  for (int axis = 0; axis < 3; ++axis) {
    if (config.gnss_sigma_override_m[axis] > 0.0) {
      sigmas[axis] = config.gnss_sigma_override_m[axis];
    }
  }
  return sigmas;
}

gtsam::SharedNoiseModel gnssNoise(
    const GnssTrajectoryOptimizer::PositionMeasurement& measurement,
    const GnssTrajectoryOptimizer::Config& config) {
  const auto gaussian = gtsam::noiseModel::Diagonal::Sigmas(
      gnssSigmas(measurement, config));
  std::string kernel = config.gnss_robust_kernel;
  std::transform(kernel.begin(), kernel.end(), kernel.begin(),
                 [](unsigned char value) { return std::tolower(value); });
  if (kernel == "none") return gaussian;
  if (kernel == "cauchy") {
    return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(config.gnss_robust_delta),
        gaussian);
  }
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(config.gnss_robust_delta), gaussian);
}

double normalizedSquaredResidual(
    const std::vector<GnssTrajectoryOptimizer::StampedPose>& poses,
    const Association& association,
    const GnssTrajectoryOptimizer::Config& config) {
  const V3 residual = interpolatePosition(
      poses, association, config.gnss_lever_arm_body_m);
  const gtsam::Vector3 sigmas = gnssSigmas(*association.measurement, config);
  return ((residual - association.measurement->position).array() /
          sigmas.array()).matrix().squaredNorm();
}

std::vector<GnssTrajectoryOptimizer::StampedPose> selectControlPoses(
    const std::vector<GnssTrajectoryOptimizer::StampedPose>& lio,
    double interval_s) {
  if (interval_s <= 0.0) return lio;
  std::vector<GnssTrajectoryOptimizer::StampedPose> controls;
  controls.reserve(static_cast<size_t>(
      std::ceil(static_cast<double>(lio.back().stamp - lio.front().stamp) * 1e-9 /
                interval_s)) + 2);
  controls.push_back(lio.front());
  const uint64_t interval_ns = static_cast<uint64_t>(std::llround(interval_s * 1e9));
  for (size_t index = 1; index + 1 < lio.size(); ++index) {
    if (lio[index].stamp - controls.back().stamp >= interval_ns) {
      controls.push_back(lio[index]);
    }
  }
  if (controls.back().stamp != lio.back().stamp) controls.push_back(lio.back());
  return controls;
}

Transform interpolateTransform(const Transform& first, const Transform& second,
                               double alpha) {
  const Quaternion rotation =
      Quaternion(first.rotation()).slerp(alpha, Quaternion(second.rotation())).normalized();
  const V3 translation =
      (1.0 - alpha) * first.translation() + alpha * second.translation();
  return Transform(rotation, translation);
}

std::vector<GnssTrajectoryOptimizer::StampedPose> applyInterpolatedCorrections(
    const std::vector<GnssTrajectoryOptimizer::StampedPose>& lio,
    const std::vector<GnssTrajectoryOptimizer::StampedPose>& control_lio,
    const std::vector<GnssTrajectoryOptimizer::StampedPose>& optimized_controls) {
  std::vector<GnssTrajectoryOptimizer::StampedPose> corrections;
  corrections.reserve(control_lio.size());
  for (size_t index = 0; index < control_lio.size(); ++index) {
    corrections.push_back(
        {control_lio[index].stamp,
         Transform(optimized_controls[index].pose * control_lio[index].pose.inverse())});
  }
  std::vector<GnssTrajectoryOptimizer::StampedPose> output;
  output.reserve(lio.size());
  size_t upper_index = 1;
  for (const auto& raw : lio) {
    while (upper_index + 1 < corrections.size() &&
           corrections[upper_index].stamp < raw.stamp) {
      ++upper_index;
    }
    const size_t lower_index = upper_index - 1;
    const uint64_t duration =
        corrections[upper_index].stamp - corrections[lower_index].stamp;
    const double alpha = duration == 0
                             ? 0.0
                             : static_cast<double>(raw.stamp -
                                                   corrections[lower_index].stamp) /
                                   static_cast<double>(duration);
    const Transform correction = interpolateTransform(
        corrections[lower_index].pose, corrections[upper_index].pose,
        std::clamp(alpha, 0.0, 1.0));
    output.push_back({raw.stamp, Transform(correction * raw.pose)});
  }
  return output;
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
      config_.vertical_sigma_floor_m <= 0.0 || config_.gnss_covariance_scale <= 0.0 ||
      config_.gnss_robust_delta <= 0.0 ||
      !std::isfinite(config_.initial_alignment_duration_s) ||
      config_.initial_alignment_duration_s <= 0.0 ||
      !std::isfinite(config_.initial_alignment_min_baseline_m) ||
      config_.initial_alignment_min_baseline_m <= 0.0 ||
      !std::isfinite(config_.initial_outlier_gate_m) ||
      config_.initial_outlier_gate_m < 0.0 ||
      config_.odom_rotation_sigma_deg_per_sqrt_m <= 0.0 ||
      config_.odom_translation_sigma_m_per_sqrt_m <= 0.0 ||
      config_.minimum_edge_distance_m <= 0.0 || config_.odom_neighbor_span <= 0 ||
      config_.control_pose_interval_s < 0.0 ||
      config_.postfit_outlier_chi2 < 0.0 || config_.max_iterations <= 0) {
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

  const std::vector<StampedPose> control_lio =
      selectControlPoses(lio, config_.control_pose_interval_s);
  stats.control_poses = control_lio.size();
  const double association_gap_s =
      std::max(config_.max_time_gap_s, config_.control_pose_interval_s);

  std::vector<Association> candidates;
  candidates.reserve(gnss.size());
  for (const PositionMeasurement& measurement : gnss) {
    Association association;
    if (measurement.position.allFinite() && measurement.variance.allFinite() &&
        associate(control_lio, measurement, association_gap_s, association)) {
      candidates.push_back(association);
    } else {
      ++stats.rejected_time;
    }
  }
  if (candidates.size() < 3) {
    if (message) *message = "Fewer than three GNSS fixes overlap the LIO trajectory.";
    return false;
  }

  // GNSS input need not be sorted. Use only the beginning of the overlap so
  // accumulated drift later in the run cannot bias the coordinate-frame seed.
  std::sort(candidates.begin(), candidates.end(),
            [](const Association& a, const Association& b) {
              return a.measurement->stamp < b.measurement->stamp;
            });
  const uint64_t alignment_start = candidates.front().measurement->stamp;
  std::vector<Association> alignment_associations;
  for (const Association& association : candidates) {
    const double elapsed_s =
        static_cast<double>(association.measurement->stamp - alignment_start) * 1e-9;
    if (elapsed_s > config_.initial_alignment_duration_s) break;
    alignment_associations.push_back(association);
  }
  if (alignment_associations.size() < 3) {
    if (message) *message =
        "Fewer than three GNSS fixes in the initial alignment window; "
        "increase initial_alignment_duration_s.";
    return false;
  }
  const V3 source_start = interpolatePosition(
      control_lio, alignment_associations.front(), config_.gnss_lever_arm_body_m);
  const V3 target_start = alignment_associations.front().measurement->position;
  double source_baseline = 0.0;
  double target_baseline = 0.0;
  for (const Association& association : alignment_associations) {
    source_baseline = std::max(source_baseline,
        (interpolatePosition(control_lio, association, config_.gnss_lever_arm_body_m) -
         source_start).head<2>().norm());
    target_baseline = std::max(target_baseline,
        (association.measurement->position - target_start).head<2>().norm());
  }
  if (std::min(source_baseline, target_baseline) <
      config_.initial_alignment_min_baseline_m) {
    if (message) *message =
        "Insufficient horizontal motion in the initial alignment window; "
        "increase initial_alignment_duration_s or check the GNSS/LIO data.";
    return false;
  }
  stats.initial_alignment_measurements = alignment_associations.size();
  stats.initial_alignment_start_offset_s =
      static_cast<double>(alignment_start - lio.front().stamp) * 1e-9;
  stats.initial_alignment_span_s = static_cast<double>(
      alignment_associations.back().measurement->stamp - alignment_start) * 1e-9;
  stats.initial_T_enu_odom = planarAlignment(
      control_lio, alignment_associations, config_.gnss_lever_arm_body_m);
  std::vector<StampedPose> initialized = control_lio;
  for (StampedPose& pose : initialized) {
    pose.pose = Transform(stats.initial_T_enu_odom * pose.pose);
  }

  std::vector<Association> associations;
  associations.reserve(candidates.size());
  std::vector<double> initial_horizontal_residuals;
  initial_horizontal_residuals.reserve(candidates.size());
  for (const Association& association : candidates) {
    const V3 residual = interpolatePosition(
                            initialized, association, config_.gnss_lever_arm_body_m) -
                        association.measurement->position;
    initial_horizontal_residuals.push_back(residual.head<2>().norm());
  }
  // Only the seed window can be judged against a rigid initial alignment.
  // Later residuals include accumulated LIO drift, so even an adaptive global
  // gate can permanently discard valid tail fixes. Admit those fixes to the
  // robust solve and judge them using the post-fit residual instead.
  const size_t seed_count = alignment_associations.size();
  const std::vector<double> seed_residuals(
      initial_horizontal_residuals.begin(),
      initial_horizontal_residuals.begin() + seed_count);
  const double residual_median = median(seed_residuals);
  std::vector<double> absolute_deviations;
  absolute_deviations.reserve(seed_residuals.size());
  for (const double residual : seed_residuals) {
    absolute_deviations.push_back(std::abs(residual - residual_median));
  }
  const double robust_sigma = 1.4826 * median(absolute_deviations);
  stats.effective_initial_outlier_gate_m = config_.initial_outlier_gate_m == 0.0
      ? 0.0
      : std::max(config_.initial_outlier_gate_m,
                 residual_median + 6.0 * std::max(robust_sigma, 0.10));
  for (size_t index = 0; index < candidates.size(); ++index) {
    const Association& association = candidates[index];
    if (index >= seed_count || config_.initial_outlier_gate_m == 0.0 ||
        initial_horizontal_residuals[index] <=
        stats.effective_initial_outlier_gate_m) {
      associations.push_back(association);
    } else {
      ++stats.rejected_outlier;
    }
  }
  if (associations.size() < 3) {
    if (message) *message = "GNSS outlier gate retained fewer than three fixes.";
    return false;
  }
  const ResidualStats before = residualStats(
      initialized, associations, config_.gnss_lever_arm_body_m);
  stats.initial_horizontal_rmse_m = before.horizontal_rmse;
  stats.initial_3d_rmse_m = before.three_dimensional_rmse;

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
  auto buildGraph = [&](const std::vector<Association>& active_associations) {
    gtsam::NonlinearFactorGraph graph;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        0, toPose3(initialized.front().pose),
        gtsam::noiseModel::Diagonal::Sigmas(prior_sigmas)));
    // GNSS positions must not bend the LIO gravity attitude over a long run.
    // Preserve each initialized pose's roll/pitch while leaving yaw and all
    // translations effectively unconstrained by these auxiliary priors.
    gtsam::Vector6 gravity_attitude_sigmas;
    gravity_attitude_sigmas
        << config_.prior_roll_pitch_sigma_deg * degrees_to_radians,
        config_.prior_roll_pitch_sigma_deg * degrees_to_radians,
        1000.0 * degrees_to_radians, 1.0e6, 1.0e6, 1.0e6;
    const auto gravity_attitude_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gravity_attitude_sigmas);
    for (size_t index = 1; index < initialized.size(); ++index) {
      graph.add(gtsam::PriorFactor<gtsam::Pose3>(
          index, toPose3(initialized[index].pose), gravity_attitude_noise));
    }
    for (size_t first = 0; first < control_lio.size(); ++first) {
      for (int step = 1; step <= config_.odom_neighbor_span; ++step) {
        const size_t second = first + static_cast<size_t>(step);
        if (second >= control_lio.size()) break;
        const gtsam::Pose3 previous = toPose3(control_lio[first].pose);
        const gtsam::Pose3 current = toPose3(control_lio[second].pose);
        const double distance = std::max(
            config_.minimum_edge_distance_m,
            (control_lio[second].pose.translation() -
             control_lio[first].pose.translation()).norm());
        const double scale = std::sqrt(distance);
        const double rotation_sigma =
            config_.odom_rotation_sigma_deg_per_sqrt_m * degrees_to_radians * scale;
        const double translation_sigma =
            config_.odom_translation_sigma_m_per_sqrt_m * scale;
        gtsam::Vector6 sigmas;
        sigmas << rotation_sigma, rotation_sigma, rotation_sigma, translation_sigma,
            translation_sigma, translation_sigma;
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            first, second, previous.between(current),
            gtsam::noiseModel::Diagonal::Sigmas(sigmas)));
      }
    }
    for (const Association& association : active_associations) {
      graph.emplace_shared<InterpolatedPositionFactor>(
          association.first, association.second, association.alpha,
          gtsam::Point3(association.measurement->position),
          gtsam::Point3(config_.gnss_lever_arm_body_m),
          gnssNoise(*association.measurement, config_));
    }
    return graph;
  };

  std::vector<StampedPose> optimized_controls;
  try {
    gtsam::NonlinearFactorGraph graph = buildGraph(associations);
    stats.initial_error = graph.error(values);
    gtsam::LevenbergMarquardtParams parameters;
    parameters.maxIterations = config_.max_iterations;
    parameters.setVerbosityLM("SILENT");
    gtsam::Values result =
        gtsam::LevenbergMarquardtOptimizer(graph, values, parameters).optimize();

    std::vector<StampedPose> first_pass;
    first_pass.reserve(control_lio.size());
    for (size_t index = 0; index < control_lio.size(); ++index) {
      first_pass.push_back(
          {control_lio[index].stamp, toTransform(result.at<gtsam::Pose3>(index))});
    }
    std::vector<Association> retained;
    retained.reserve(associations.size());
    size_t rejected_postfit = 0;
    for (const Association& association : associations) {
      if (config_.postfit_outlier_chi2 == 0.0 ||
          normalizedSquaredResidual(first_pass, association, config_) <=
              config_.postfit_outlier_chi2) {
        retained.push_back(association);
      } else {
        ++rejected_postfit;
      }
    }
    if (retained.size() >= 3 && retained.size() != associations.size()) {
      graph = buildGraph(retained);
      result = gtsam::LevenbergMarquardtOptimizer(graph, result, parameters).optimize();
      associations = std::move(retained);
      stats.rejected_postfit = rejected_postfit;
    }
    stats.used_measurements = associations.size();
    stats.final_error = graph.error(result);
    optimized_controls.reserve(control_lio.size());
    for (size_t index = 0; index < control_lio.size(); ++index) {
      optimized_controls.push_back(
          {control_lio[index].stamp, toTransform(result.at<gtsam::Pose3>(index))});
    }
    optimized = applyInterpolatedCorrections(lio, control_lio, optimized_controls);
  } catch (const std::exception& error) {
    if (message) *message = std::string("GTSAM optimization failed: ") + error.what();
    return false;
  }

  const ResidualStats after = residualStats(
      optimized_controls, associations, config_.gnss_lever_arm_body_m);
  stats.optimized_horizontal_rmse_m = after.horizontal_rmse;
  stats.optimized_3d_rmse_m = after.three_dimensional_rmse;
  if (message) {
    std::ostringstream stream;
    stream << "Optimized " << stats.control_poses << " control poses and expanded to "
           << optimized.size() << " LIO poses with "
           << stats.used_measurements << " GNSS factors; horizontal RMSE "
           << stats.initial_horizontal_rmse_m << " -> "
           << stats.optimized_horizontal_rmse_m << " m.";
    *message = stream.str();
  }
  return true;
}

}  // namespace bievr
