#include "bievr_pgo/loop_closer.h"

#include <small_gicp/registration/registration_helper.hpp>

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_set>

#include "bievr_lio/utils.h"
#include "bievr_lio/log++.h"

namespace bievr {
namespace {

gtsam::Pose3 toPose3(const Transform& t) {
  return gtsam::Pose3(gtsam::Rot3(t.linear()), gtsam::Point3(t.translation()));
}

Transform toTransform(const gtsam::Pose3& p) {
  Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
  iso.linear() = p.rotation().matrix();
  iso.translation() = p.translation();
  return Transform(iso);
}

Eigen::Matrix4f toMatrix4f(const gtsam::Pose3& p) { return p.matrix().cast<float>(); }

double deg2rad(double deg) { return deg * M_PI / 180.0; }

LoopCloser::Cloud::Ptr voxelDownsample(const LoopCloser::Cloud::ConstPtr& input, double leaf) {
  LoopCloser::Cloud::Ptr output(new LoopCloser::Cloud());
  if (!input || input->empty()) return output;
  if (leaf <= 0.0) {
    *output = *input;
    return output;
  }
  output->reserve(input->size());
  std::unordered_set<size_t> occupied;
  occupied.reserve(input->size());
  const double inverse = 1.0 / leaf;
  for (const auto& p : input->points) {
    const Eigen::Vector3i voxel =
        (Eigen::Vector3d(p.x, p.y, p.z) * inverse).array().floor().cast<int>();
    if (occupied.insert(hashIndexVoxel(voxel)).second) output->push_back(p);
  }
  output->is_dense = true;
  return output;
}

void appendTransformed(const LoopCloser::Cloud& input, const Eigen::Matrix4f& transform,
                       LoopCloser::Cloud& output) {
  output.reserve(output.size() + input.size());
  for (const auto& p : input.points) {
    const Eigen::Vector4f moved = transform * Eigen::Vector4f(p.x, p.y, p.z, 1.0f);
    output.push_back(LoopCloser::PointT(moved.x(), moved.y(), moved.z()));
  }
}

std::vector<Eigen::Vector3d> toEigenPoints(const LoopCloser::Cloud& cloud) {
  std::vector<Eigen::Vector3d> points;
  points.reserve(cloud.size());
  for (const auto& p : cloud.points) points.emplace_back(p.x, p.y, p.z);
  return points;
}

}  // namespace

LoopCloser::LoopCloser(Config config)
    : config_(std::move(config)),
      keyframe_rad_gap_(deg2rad(config_.keyframe_deg_gap)),
      scan_context_(config_.scan_context) {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = config_.isam_relinearize_threshold;
  params.relinearizeSkip = config_.isam_relinearize_skip;
  isam_ = std::make_unique<gtsam::ISAM2>(params);

  initNoiseModels();

  workers_.emplace_back(&LoopCloser::keyframeWorker, this);
  workers_.emplace_back(&LoopCloser::loopDetectWorker, this);
  workers_.emplace_back(&LoopCloser::icpWorker, this);
  workers_.emplace_back(&LoopCloser::isamWorker, this);
}

std::unique_ptr<LoopCloser> LoopCloser::Create(Config config) {
  return std::unique_ptr<LoopCloser>(new LoopCloser(std::move(config)));
}

LoopCloser::~LoopCloser() {
  {
    std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_ = true;
  }
  // Take each waiter's mutex once so none can be between its predicate check
  // and its wait when the notification goes out.
  { std::lock_guard<std::mutex> lock(queue_mutex_); }
  { std::lock_guard<std::mutex> lock(candidate_mutex_); }
  stop_cv_.notify_all();
  queue_cv_.notify_all();
  candidate_cv_.notify_all();

  for (auto& worker : workers_) {
    if (worker.joinable()) worker.join();
  }
  workers_.clear();

  // Release the large cross-library objects explicitly while still inside the
  // PGO DSO. This is also easier to keep ABI-safe with GTSAM 4.0.3/PCL 1.10
  // than relying on consumer-side inlined cleanup during process shutdown.
  {
    std::lock_guard<std::mutex> lock(candidate_mutex_);
    candidates_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    keyframe_stamps_.clear();
    keyframe_poses_.clear();
    keyframe_odom_poses_.clear();
    keyframe_clouds_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(sc_mutex_);
    scan_context_.clear();
  }
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    initial_estimate_.clear();
    graph_.resize(0);
    isam_.reset();
  }
  loop_noise_.reset();
  odom_noise_.reset();
  prior_noise_.reset();
}

bool LoopCloser::waitOrStop(std::chrono::duration<double> period) {
  std::unique_lock<std::mutex> lock(stop_mutex_);
  return stop_cv_.wait_for(lock, period, [this] { return stop_.load(); });
}

void LoopCloser::initNoiseModels() {
  // gtsam::Pose3's tangent space is [rotation, translation].
  gtsam::Vector6 prior_variance;
  prior_variance.setConstant(config_.prior_noise_score);
  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(prior_variance);

  gtsam::Vector6 odom_variance;
  odom_variance << config_.odom_noise_rotation, config_.odom_noise_rotation,
      config_.odom_noise_rotation, config_.odom_noise_translation,
      config_.odom_noise_translation, config_.odom_noise_translation;
  odom_noise_ = gtsam::noiseModel::Diagonal::Variances(odom_variance);

  gtsam::Vector6 loop_variance;
  loop_variance.setConstant(config_.loop_noise_score);
  loop_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(config_.loop_noise_cauchy_c),
      gtsam::noiseModel::Diagonal::Variances(loop_variance));
}

void LoopCloser::addFrame(uint64_t stamp, const Transform& T_W_I, const Pointcloud& cloud_body) {
  // Gate here, on the odometry thread: it is a pose delta only, and it saves
  // copying the clouds of frames that will not become keyframes.
  bool is_keyframe = !has_previous_frame_;
  if (has_previous_frame_) {
    const Eigen::Isometry3d delta = previous_pose_.inverse() * T_W_I;
    translation_accumulated_ += delta.translation().norm();
    rotation_accumulated_ += Eigen::AngleAxisd(delta.linear()).angle();
    is_keyframe = translation_accumulated_ > config_.keyframe_meter_gap ||
                  rotation_accumulated_ > keyframe_rad_gap_;
  }
  previous_pose_ = T_W_I;
  has_previous_frame_ = true;
  if (!is_keyframe) return;

  translation_accumulated_ = 0.0;
  rotation_accumulated_ = 0.0;

  PendingFrame frame;
  frame.stamp = stamp;
  frame.pose = T_W_I;
  frame.cloud = Cloud::Ptr(new Cloud());
  const auto& data = cloud_body.data();
  frame.cloud->reserve(data.cols());
  for (Eigen::Index i = 0; i < data.cols(); ++i) {
    const float x = static_cast<float>(data(0, i));
    const float y = static_cast<float>(data(1, i));
    const float z = static_cast<float>(data(2, i));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;
    frame.cloud->push_back(PointT(x, y, z));
  }
  frame.cloud->is_dense = true;
  if (frame.cloud->empty()) return;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= config_.max_queue) {
      queue_.pop_front();
      ++dropped_frames_;
    }
    queue_.push_back(std::move(frame));
  }
  queue_cv_.notify_one();
}

void LoopCloser::keyframeWorker() {
  while (true) {
    PendingFrame frame;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
      if (stop_ && queue_.empty()) return;
      frame = std::move(queue_.front());
      queue_.pop_front();
    }
    integrateKeyframe(std::move(frame));
  }
}

void LoopCloser::integrateKeyframe(PendingFrame frame) {
  Cloud::Ptr downsampled = voxelDownsample(frame.cloud, config_.keyframe_filter_size);

  const gtsam::Pose3 pose = toPose3(frame.pose);

  int index = 0;
  gtsam::Pose3 previous_odom_pose;
  {
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    index = static_cast<int>(keyframe_clouds_.size());
    if (index > 0) previous_odom_pose = keyframe_odom_poses_.back();
    keyframe_clouds_.push_back(downsampled);
    keyframe_odom_poses_.push_back(pose);
    keyframe_poses_.push_back(pose);
    keyframe_stamps_.push_back(frame.stamp);
  }

  {
    // Scan Context wants the body-frame cloud, at the same resolution the
    // original pipeline fed it.
    Eigen::Matrix<double, 3, Eigen::Dynamic> sc_cloud(3, downsampled->size());
    for (size_t i = 0; i < downsampled->size(); ++i) {
      const auto& p = downsampled->points[i];
      sc_cloud.col(i) << p.x, p.y, p.z;
    }
    std::lock_guard<std::mutex> lock(sc_mutex_);
    scan_context_.add(sc_cloud);
  }

  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (!graph_initialized_) {
    graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, pose, prior_noise_));
    initial_estimate_.insert(0, pose);
    graph_initialized_ = true;
  } else {
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(index - 1, index,
                                                  previous_odom_pose.between(pose), odom_noise_));
    initial_estimate_.insert(index, pose);
  }
}

void LoopCloser::loopDetectWorker() {
  const auto period = std::chrono::duration<double>(1.0 / config_.loop_detect_frequency);
  int last_queried = -1;

  while (true) {
    if (waitOrStop(period)) return;

    // detectLoop() always queries with the newest descriptor, so re-running it
    // without a new keyframe would re-add the same constraint every cycle.
    ScanContext::Match match;
    int current = -1;
    {
      std::lock_guard<std::mutex> lock(sc_mutex_);
      current = static_cast<int>(scan_context_.size()) - 1;
      if (current < 0 || current == last_queried) continue;
      last_queried = current;
      match = scan_context_.detectLoop();
    }
    if (match.index < 0) continue;

    {
      std::lock_guard<std::mutex> lock(candidate_mutex_);
      candidates_.push_back({match.index, current, match.yaw_diff_rad, match.distance});
    }
    candidate_cv_.notify_one();
  }
}

void LoopCloser::icpWorker() {
  while (true) {
    LoopCandidate candidate;
    {
      std::unique_lock<std::mutex> lock(candidate_mutex_);
      candidate_cv_.wait(lock, [this] { return stop_ || !candidates_.empty(); });
      if (candidates_.empty()) return;  // predicate guarantees this means stop_
      candidate = candidates_.front();
      candidates_.pop_front();
    }

    const auto constraint = computeLoopConstraint(candidate);
    if (!constraint) {
      ++num_rejected_;
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(candidate.from, candidate.to, *constraint,
                                                    loop_noise_));
    }
    ++num_loops_;
  }
}

LoopCloser::Cloud::Ptr LoopCloser::buildSubmap(int key, int span) const {
  std::vector<std::pair<Cloud::Ptr, gtsam::Pose3>> selected;
  {
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    const int count = static_cast<int>(keyframe_clouds_.size());
    for (int i = -span; i <= span; ++i) {
      const int near = key + i;
      if (near < 0 || near >= count) continue;
      selected.emplace_back(keyframe_clouds_[near], keyframe_poses_[near]);
    }
  }

  // Each cloud goes at its own pose, so the submap is geometrically coherent and
  // ICP's output is a world-frame correction rather than a relative transform.
  Cloud::Ptr submap(new Cloud());
  for (const auto& [cloud, pose] : selected) {
    appendTransformed(*cloud, toMatrix4f(pose), *submap);
  }
  if (submap->empty()) return submap;
  return voxelDownsample(submap, config_.icp_filter_size);
}

std::optional<gtsam::Pose3> LoopCloser::computeLoopConstraint(const LoopCandidate& candidate) {
  std::lock_guard<std::mutex> pcl_lock(pcl_mutex_);
  Cloud::Ptr source = buildSubmap(candidate.to, 0);
  Cloud::Ptr target = buildSubmap(candidate.from, config_.history_keyframe_search_num);
  if (source->empty() || target->empty()) return std::nullopt;

  gtsam::Pose3 pose_curr, pose_loop;
  {
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    pose_curr = keyframe_poses_.at(candidate.to);
    pose_loop = keyframe_poses_.at(candidate.from);
  }

  // Both clouds are in the world frame, so identity is the right start unless the
  // revisit runs in the opposite direction. Then Scan Context's yaw is lifted to
  // the world frame as P_loop * Rz(+-yaw) * P_curr^-1; the sign is not measured,
  // so both are tried. That best-of-two also weakens the fitness rejection below,
  // which is why it is off by default.
  std::vector<Eigen::Matrix4f> guesses;
  const double yaw = candidate.yaw_diff_rad;
  if (config_.use_sc_yaw_guess &&
      std::abs(yaw) >= std::max(deg2rad(config_.sc_yaw_guess_min_deg), 1e-6)) {
    for (const double sign : {-1.0, 1.0}) {
      const gtsam::Pose3 rotation(gtsam::Rot3::Yaw(sign * yaw), gtsam::Point3(0, 0, 0));
      guesses.push_back(toMatrix4f(pose_loop * rotation * pose_curr.inverse()));
    }
  } else {
    guesses.push_back(Eigen::Matrix4f::Identity());
  }

  // Scan Context proposes the place; one small_gicp pass estimates the full
  // six-DoF loop transform. Both point sets already live in the odometry world
  // frame, so the result is a world-frame correction of the current keyframe.
  const auto [gicp_target, target_tree] = small_gicp::preprocess_points(
      toEigenPoints(*target), config_.icp_filter_size, 20, 4);
  const auto [gicp_source, unused_source_tree] = small_gicp::preprocess_points(
      toEigenPoints(*source), config_.icp_filter_size, 20, 4);
  if (!gicp_target || !target_tree || !gicp_source || gicp_target->empty() ||
      gicp_source->empty()) {
    return std::nullopt;
  }

  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.max_correspondence_distance = config_.icp_max_correspondence_distance;
  setting.rotation_eps =
      std::sqrt(std::max(config_.icp_transformation_epsilon, 1e-12));
  setting.translation_eps =
      std::sqrt(std::max(config_.icp_transformation_epsilon, 1e-12));
  setting.max_iterations = config_.icp_max_iterations;
  setting.num_threads = 4;

  bool converged = false;
  double best_fitness = std::numeric_limits<double>::max();
  double best_inlier_ratio = 0.0;
  Eigen::Matrix4f best_transformation = Eigen::Matrix4f::Identity();
  const double max_distance_sq = config_.icp_max_correspondence_distance *
                                 config_.icp_max_correspondence_distance;

  for (const auto& guess : guesses) {
    const auto aligned = small_gicp::align(
        *gicp_target, *gicp_source, *target_tree,
        Eigen::Isometry3d(guess.cast<double>()), setting);
    if (!aligned.converged || aligned.num_inliers < 20) continue;

    double squared_error = 0.0;
    size_t inliers = 0;
    for (const auto& raw : source->points) {
      const Eigen::Vector4d query = aligned.T_target_source *
                                    Eigen::Vector4d(raw.x, raw.y, raw.z, 1.0);
      size_t index = 0;
      double distance = 0.0;
      if (target_tree->nearest_neighbor_search(query, &index, &distance) == 1 &&
          distance <= max_distance_sq) {
        squared_error += distance;
        ++inliers;
      }
    }
    if (inliers < 20) continue;
    const double fitness = squared_error / static_cast<double>(inliers);
    if (fitness < best_fitness) {
      converged = true;
      best_fitness = fitness;
      best_inlier_ratio = static_cast<double>(inliers) / source->size();
      best_transformation = aligned.T_target_source.matrix().cast<float>();
    }
  }

  if (!converged || best_fitness > config_.loop_fitness_score_threshold) {
    LOG(I, "Rejected loop " << candidate.from << " -> " << candidate.to
                             << ", SC " << candidate.sc_distance << ", ICP "
                             << best_fitness << ", inliers "
                             << best_inlier_ratio * 100.0 << "%.");
    return std::nullopt;
  }
  // Evaluate Eigen expressions containing commas before passing them to LOG;
  // otherwise the variadic logging macro interprets template commas as
  // additional macro arguments.
  const double correction_translation =
      best_transformation.topRightCorner<3, 1>().norm();
  const double correction_rotation_deg =
      Eigen::AngleAxisf(best_transformation.topLeftCorner<3, 3>()).angle() *
      180.0 / M_PI;
  LOG(I, "Accepted loop " << candidate.from << " -> " << candidate.to
                           << ", SC " << candidate.sc_distance << ", ICP "
                           << best_fitness << ", inliers "
                           << best_inlier_ratio * 100.0 << "%, correction "
                           << correction_translation << " m / "
                           << correction_rotation_deg << " deg.");

  // getFinalTransformation() includes the initial guess, and corrects the current
  // keyframe in the world frame: P_corrected = T_icp * P_curr. BetweenFactor
  // wants P_loop^-1 * P_corrected.
  const gtsam::Pose3 correction(best_transformation.cast<double>());
  return pose_loop.between(correction * pose_curr);
}

void LoopCloser::isamWorker() {
  const auto period = std::chrono::duration<double>(1.0 / config_.isam_frequency);
  while (true) {
    if (waitOrStop(period)) return;
    optimize();
  }
}

void LoopCloser::optimize() {
  gtsam::Values estimate;
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (!graph_initialized_ || (graph_.empty() && initial_estimate_.empty())) return;

    isam_->update(graph_, initial_estimate_);
    isam_->update();
    graph_.resize(0);
    initial_estimate_.clear();
    estimate = isam_->calculateEstimate();
  }

  std::lock_guard<std::mutex> lock(keyframe_mutex_);
  for (size_t i = 0; i < keyframe_poses_.size() && i < estimate.size(); ++i) {
    keyframe_poses_[i] = estimate.at<gtsam::Pose3>(i);
  }
}

LoopCloser::Snapshot LoopCloser::snapshot() const {
  std::lock_guard<std::mutex> lock(keyframe_mutex_);
  // Clouds are shared_ptr copies; the workers only ever append, never mutate.
  return Snapshot{keyframe_clouds_, keyframe_poses_, keyframe_stamps_};
}

LoopCloser::Cloud::Ptr LoopCloser::accumulate(const Snapshot& snap, double resolution) const {
  Cloud::Ptr map(new Cloud());
  for (size_t i = 0; i < snap.clouds.size() && i < snap.poses.size(); ++i) {
    appendTransformed(*snap.clouds[i], toMatrix4f(snap.poses[i]), *map);
  }
  if (resolution <= 0.0 || map->empty()) return map;
  return voxelDownsample(map, resolution);
}

Pointcloud LoopCloser::buildMap(double resolution) const {
  const Cloud::Ptr map = accumulate(snapshot(), resolution);
  Pointcloud out;
  out.data().resize(3, map->size());
  for (size_t i = 0; i < map->size(); ++i) {
    const auto& p = map->points[i];
    out.data().col(i) << p.x, p.y, p.z;
  }
  return out;
}

std::vector<LoopCloser::Keyframe> LoopCloser::keyframes() const {
  std::lock_guard<std::mutex> lock(keyframe_mutex_);
  std::vector<Keyframe> out(keyframe_poses_.size());
  for (size_t i = 0; i < keyframe_poses_.size(); ++i) {
    out[i].stamp = keyframe_stamps_[i];
    out[i].pose = toTransform(keyframe_poses_[i]);
  }
  return out;
}

bool LoopCloser::idle() const {
  // Taken in sequence, never nested, to keep the lock order free of cycles.
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!queue_.empty()) return false;
  }
  {
    std::lock_guard<std::mutex> lock(candidate_mutex_);
    if (!candidates_.empty()) return false;
  }
  std::lock_guard<std::mutex> lock(graph_mutex_);
  return graph_.empty() && initial_estimate_.empty();
}

LoopCloser::Stats LoopCloser::stats() const {
  Stats stats;
  {
    std::lock_guard<std::mutex> lock(keyframe_mutex_);
    stats.num_keyframes = keyframe_poses_.size();
  }
  stats.num_loops = num_loops_;
  stats.num_rejected = num_rejected_;
  stats.dropped_frames = dropped_frames_;
  return stats;
}

bool LoopCloser::saveMapBundle(const std::string& dir, std::string* message) const {
  const auto fail = [message](const std::string& text) {
    if (message) *message = text;
    return false;
  };

  // Snapshot the keyframes and the descriptors back to back. They are appended
  // under different locks, so any work between these two lines desynchronises
  // the pose/descriptor index alignment that relocalization reads them by.
  const Snapshot snap = snapshot();
  ScanContext::Archive descriptors;
  {
    std::lock_guard<std::mutex> lock(sc_mutex_);
    descriptors = scan_context_.archive();
  }
  if (snap.clouds.empty()) return fail("No keyframes accumulated yet.");

  std::vector<Transform> poses;
  poses.reserve(snap.poses.size());
  for (const gtsam::Pose3& pose : snap.poses) poses.push_back(toTransform(pose));

  MapMeta meta;
  meta.frame = config_.map_frame;
  meta.map_save_resolution_m = config_.map_save_resolution;
  meta.keyframe_filter_size_m = config_.keyframe_filter_size;
  meta.scan_context = config_.scan_context;

  const Cloud::Ptr map = accumulate(snap, config_.map_save_resolution);
  return bievr::saveMapBundle(dir, *map, poses, snap.stamps, descriptors, meta, message);
}

}  // namespace bievr
