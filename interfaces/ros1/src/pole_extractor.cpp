#include "bievr_lio_ros/pole_extractor.h"

#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "bievr_lio/config_loader.h"
#include "bievr_lio/log++.h"
#include "bievr_ros_common/conversions.h"

namespace bievr {
namespace {

struct PolePoint {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

using PoleCloud = std::vector<PolePoint>;

struct Cell {
  int x = 0;
  int y = 0;

  bool operator==(const Cell& other) const { return x == other.x && y == other.y; }
};

struct CellHash {
  size_t operator()(const Cell& cell) const {
    const uint64_t ux = static_cast<uint32_t>(cell.x);
    const uint64_t uy = static_cast<uint32_t>(cell.y);
    uint64_t value = (ux << 32U) | uy;
    value ^= value >> 33U;
    value *= 0xff51afd7ed558ccdULL;
    value ^= value >> 33U;
    return static_cast<size_t>(value);
  }
};

struct Voxel {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator==(const Voxel& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash {
  size_t operator()(const Voxel& voxel) const {
    size_t seed = CellHash()({voxel.x, voxel.y});
    seed ^= static_cast<size_t>(static_cast<uint32_t>(voxel.z)) + 0x9e3779b9U + (seed << 6U) +
            (seed >> 2U);
    return seed;
  }
};

struct PointSum {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  size_t count = 0;
};

struct GridStat {
  int count = 0;
  float z_min = std::numeric_limits<float>::infinity();
  float z_max = -std::numeric_limits<float>::infinity();
  std::unordered_set<int> layers;
};

struct ExtractionFrame {
  uint64_t stamp = 0;
  Transform pose;
  Pointcloud cloud;
};

struct ExtractionTask {
  size_t index = 0;
  std::vector<ExtractionFrame> frames;
};

struct Detection {
  size_t keyframe = 0;
  uint64_t stamp = 0;
  V3 center = V3::Zero();
  V3 axis = V3::UnitZ();
  double radius = 0.0;
  double cluster_radius = 0.0;
  double fit_rms = 0.0;
  double inlier_ratio = 0.0;
  double angular_coverage = 0.0;
  double axis_tilt_deg = 0.0;
  double axial_min = 0.0;
  double axial_max = 0.0;
  size_t cylinder_segments = 1;
  double span_x = 0.0;
  double span_y = 0.0;
  size_t points = 0;
};

struct Landmark {
  size_t id = 0;
  V3 center = V3::Zero();
  V3 axis = V3::UnitZ();
  double radius = 0.0;
  double fit_rms = 0.0;
  double axial_min = 0.0;
  double axial_max = 0.0;
  size_t observations = 0;
  size_t total_points = 0;
};

struct CylinderModel {
  V3 axis = V3::UnitZ();
  V3 axis_point = V3::Zero();
  double radius = 0.0;
  double rms = std::numeric_limits<double>::infinity();
  double inlier_ratio = 0.0;
  double angular_coverage = 0.0;
  double tilt_deg = 90.0;
  size_t point_count = 0;
  bool pca_valid = false;
  bool valid = false;
};

struct ExtractionResult {
  PoleCloud downsampled;
  PoleCloud candidates;
  PoleCloud refined;
  std::vector<Detection> detections;
};

Cell toCell(const PolePoint& point, double resolution) {
  return {static_cast<int>(std::floor(static_cast<double>(point.x) / resolution)),
          static_cast<int>(std::floor(static_cast<double>(point.y) / resolution))};
}

int longestContinuousRun(const std::unordered_set<int>& layer_set) {
  if (layer_set.empty()) return 0;
  std::vector<int> layers(layer_set.begin(), layer_set.end());
  std::sort(layers.begin(), layers.end());
  int longest = 1;
  int current = 1;
  for (size_t i = 1; i < layers.size(); ++i) {
    if (layers[i] == layers[i - 1] + 1) {
      longest = std::max(longest, ++current);
    } else {
      current = 1;
    }
  }
  return longest;
}

std::unordered_set<Cell, CellHash> closeMask(
    const std::unordered_set<Cell, CellHash>& candidates) {
  std::unordered_set<Cell, CellHash> dilated;
  dilated.reserve(candidates.size() * 9);
  for (const Cell& cell : candidates) {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        dilated.insert({cell.x + dx, cell.y + dy});
      }
    }
  }

  std::unordered_set<Cell, CellHash> closed;
  closed.reserve(dilated.size());
  for (const Cell& cell : dilated) {
    bool keep = true;
    for (int dx = -1; dx <= 1 && keep; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dilated.find({cell.x + dx, cell.y + dy}) == dilated.end()) {
          keep = false;
          break;
        }
      }
    }
    if (keep) closed.insert(cell);
  }
  return closed;
}

std::unordered_set<Cell, CellHash> smallComponents(
    const std::unordered_set<Cell, CellHash>& mask, int max_component_cells) {
  std::unordered_set<Cell, CellHash> visited;
  std::unordered_set<Cell, CellHash> valid;
  visited.reserve(mask.size());
  valid.reserve(mask.size());

  for (const Cell& seed : mask) {
    if (visited.find(seed) != visited.end()) continue;
    std::vector<Cell> component;
    std::deque<Cell> frontier;
    frontier.push_back(seed);
    visited.insert(seed);
    while (!frontier.empty()) {
      const Cell cell = frontier.front();
      frontier.pop_front();
      component.push_back(cell);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          const Cell neighbor{cell.x + dx, cell.y + dy};
          if (mask.find(neighbor) != mask.end() && visited.insert(neighbor).second) {
            frontier.push_back(neighbor);
          }
        }
      }
    }
    if (component.size() <= static_cast<size_t>(max_component_cells)) {
      valid.insert(component.begin(), component.end());
    }
  }
  return valid;
}

Pointcloud toBievrCloud(const PoleCloud& cloud) {
  Pointcloud result;
  result.resize(cloud.size());
  for (size_t i = 0; i < cloud.size(); ++i) {
    result.data().col(static_cast<Eigen::Index>(i)) =
        V3(cloud[i].x, cloud[i].y, cloud[i].z);
  }
  return result;
}

Pointcloud centersToCloud(const std::vector<Detection>& detections) {
  Pointcloud cloud;
  cloud.resize(detections.size());
  for (size_t i = 0; i < detections.size(); ++i) {
    cloud.data().col(static_cast<Eigen::Index>(i)) = detections[i].center;
  }
  return cloud;
}

Pointcloud landmarksToCloud(const std::vector<Landmark>& landmarks) {
  Pointcloud cloud;
  cloud.resize(landmarks.size());
  for (size_t i = 0; i < landmarks.size(); ++i) {
    cloud.data().col(static_cast<Eigen::Index>(i)) = landmarks[i].center;
  }
  return cloud;
}

bool writeXYZPcd(const std::filesystem::path& path, const PoleCloud& cloud) {
  std::ofstream file(path, std::ios::binary);
  if (!file) return false;
  file << "# .PCD v0.7 - Point Cloud Data file format\n"
       << "VERSION 0.7\n"
       << "FIELDS x y z\n"
       << "SIZE 4 4 4\n"
       << "TYPE F F F\n"
       << "COUNT 1 1 1\n"
       << "WIDTH " << cloud.size() << "\n"
       << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS " << cloud.size() << "\n"
       << "DATA binary\n";
  for (const PolePoint& point : cloud) {
    file.write(reinterpret_cast<const char*>(&point.x), sizeof(float));
    file.write(reinterpret_cast<const char*>(&point.y), sizeof(float));
    file.write(reinterpret_cast<const char*>(&point.z), sizeof(float));
  }
  return static_cast<bool>(file);
}

bool writeLandmarkPcd(const std::filesystem::path& path,
                      const std::vector<Landmark>& landmarks) {
  std::ofstream file(path, std::ios::binary);
  if (!file) return false;
  file << "# .PCD v0.7 - Point Cloud Data file format\n"
       << "VERSION 0.7\n"
       << "FIELDS x y z intensity\n"
       << "SIZE 4 4 4 4\n"
       << "TYPE F F F F\n"
       << "COUNT 1 1 1 1\n"
       << "WIDTH " << landmarks.size() << "\n"
       << "HEIGHT 1\n"
       << "VIEWPOINT 0 0 0 1 0 0 0\n"
       << "POINTS " << landmarks.size() << "\n"
       << "DATA binary\n";
  for (const Landmark& landmark : landmarks) {
    const float values[4] = {static_cast<float>(landmark.center.x()),
                             static_cast<float>(landmark.center.y()),
                             static_cast<float>(landmark.center.z()),
                             static_cast<float>(landmark.observations)};
    file.write(reinterpret_cast<const char*>(values), sizeof(values));
  }
  return static_cast<bool>(file);
}

PoleCloud voxelDownsample(PoleCloud input, double resolution) {
  if (resolution <= 0.0 || input.empty()) return input;
  std::unordered_map<Voxel, PointSum, VoxelHash> voxels;
  voxels.reserve(input.size() / 4 + 1);
  for (const PolePoint& point : input) {
    const Voxel voxel{
        static_cast<int>(std::floor(static_cast<double>(point.x) / resolution)),
        static_cast<int>(std::floor(static_cast<double>(point.y) / resolution)),
        static_cast<int>(std::floor(static_cast<double>(point.z) / resolution))};
    PointSum& sum = voxels[voxel];
    sum.x += point.x;
    sum.y += point.y;
    sum.z += point.z;
    ++sum.count;
  }
  PoleCloud output;
  output.reserve(voxels.size());
  for (const auto& [voxel, sum] : voxels) {
    const double count = static_cast<double>(sum.count);
    output.push_back({static_cast<float>(sum.x / count), static_cast<float>(sum.y / count),
                      static_cast<float>(sum.z / count)});
  }
  return output;
}

std::vector<Eigen::Vector2d> trajectoryTangents(const std::vector<ExtractionFrame>& frames) {
  std::vector<Eigen::Vector2d> tangents(frames.size(), Eigen::Vector2d::UnitX());
  for (size_t i = 0; i < frames.size(); ++i) {
    Eigen::Vector2d tangent;
    if (frames.size() > 1) {
      const size_t before = i == 0 ? 0 : i - 1;
      const size_t after = i + 1 < frames.size() ? i + 1 : i;
      tangent = (frames[after].pose.translation() - frames[before].pose.translation()).head<2>();
    }
    if (frames.size() == 1 || tangent.norm() < 1e-6) {
      tangent = frames[i].pose.rotation().block<2, 1>(0, 0);
    }
    if (tangent.norm() < 1e-6) tangent = Eigen::Vector2d::UnitX();
    tangents[i] = tangent.normalized();
  }
  return tangents;
}

PoleCloud accumulateFrames(const std::vector<ExtractionFrame>& frames,
                           const PoleExtractionConfig& config) {
  PoleCloud cloud;
  size_t reserve = 0;
  for (const auto& frame : frames) reserve += frame.cloud.size();
  cloud.reserve(reserve);

  const auto tangents = trajectoryTangents(frames);
  for (size_t frame_index = 0; frame_index < frames.size(); ++frame_index) {
    const ExtractionFrame& frame = frames[frame_index];
    const Eigen::Vector2d left(-tangents[frame_index].y(), tangents[frame_index].x());
    for (size_t i = 0; i < frame.cloud.size(); ++i) {
      const V3 local = frame.cloud[i];
      if (!local.allFinite()) continue;
      const V3 world = frame.pose * local;
      if (config.use_roi) {
        const Eigen::Vector2d rel_xy =
            world.head<2>() - frame.pose.translation().head<2>();
        const double rel_y = rel_xy.dot(left);
        const double rel_z = world.z() - frame.pose.translation().z();
        if (rel_y < config.roi_y_min || rel_y > config.roi_y_max ||
            rel_z < config.roi_z_min || rel_z > config.roi_z_max) {
          continue;
        }
      }
      cloud.push_back({static_cast<float>(world.x()), static_cast<float>(world.y()),
                       static_cast<float>(world.z())});
    }
  }
  return cloud;
}

double nearestTrajectoryZ(double x, double y, const std::vector<ExtractionFrame>& frames) {
  double best_distance_sq = std::numeric_limits<double>::infinity();
  double z = 0.0;
  for (const auto& frame : frames) {
    const double dx = x - frame.pose.translation().x();
    const double dy = y - frame.pose.translation().y();
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      z = frame.pose.translation().z();
    }
  }
  return z;
}

std::vector<std::vector<size_t>> euclideanClusters(const PoleCloud& cloud, double tolerance) {
  std::unordered_map<Voxel, std::vector<size_t>, VoxelHash> buckets;
  buckets.reserve(cloud.size() / 4 + 1);
  auto voxelOf = [tolerance](const PolePoint& point) {
    return Voxel{static_cast<int>(std::floor(static_cast<double>(point.x) / tolerance)),
                 static_cast<int>(std::floor(static_cast<double>(point.y) / tolerance)),
                 static_cast<int>(std::floor(static_cast<double>(point.z) / tolerance))};
  };
  for (size_t i = 0; i < cloud.size(); ++i) buckets[voxelOf(cloud[i])].push_back(i);

  const double tolerance_sq = tolerance * tolerance;
  std::vector<uint8_t> visited(cloud.size(), 0);
  std::vector<std::vector<size_t>> clusters;
  for (size_t seed = 0; seed < cloud.size(); ++seed) {
    if (visited[seed]) continue;
    visited[seed] = 1;
    std::vector<size_t> cluster;
    std::deque<size_t> frontier;
    frontier.push_back(seed);
    while (!frontier.empty()) {
      const size_t current = frontier.front();
      frontier.pop_front();
      cluster.push_back(current);
      const PolePoint& point = cloud[current];
      const Voxel voxel = voxelOf(point);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const auto bucket = buckets.find({voxel.x + dx, voxel.y + dy, voxel.z + dz});
            if (bucket == buckets.end()) continue;
            for (size_t neighbor : bucket->second) {
              if (visited[neighbor]) continue;
              const PolePoint& other = cloud[neighbor];
              const double px = static_cast<double>(point.x) - other.x;
              const double py = static_cast<double>(point.y) - other.y;
              const double pz = static_cast<double>(point.z) - other.z;
              if (px * px + py * py + pz * pz <= tolerance_sq) {
                visited[neighbor] = 1;
                frontier.push_back(neighbor);
              }
            }
          }
        }
      }
    }
    clusters.push_back(std::move(cluster));
  }
  return clusters;
}

struct CirclePoint {
  double x = 0.0;
  double y = 0.0;
};

bool circleFromThree(const CirclePoint& a, const CirclePoint& b, const CirclePoint& c,
                     double& center_x, double& center_y, double& radius) {
  const double denominator =
      2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
  if (std::abs(denominator) < 1e-10) return false;
  const double a2 = a.x * a.x + a.y * a.y;
  const double b2 = b.x * b.x + b.y * b.y;
  const double c2 = c.x * c.x + c.y * c.y;
  center_x = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) /
             denominator;
  center_y = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) /
             denominator;
  radius = std::hypot(a.x - center_x, a.y - center_y);
  return std::isfinite(radius) && radius > 1e-6;
}

bool leastSquaresCircle(const std::vector<CirclePoint>& points,
                        const std::vector<size_t>& inliers, double& center_x,
                        double& center_y, double& radius) {
  if (inliers.size() < 3) return false;
  Eigen::MatrixXd system(static_cast<Eigen::Index>(inliers.size()), 3);
  Eigen::VectorXd rhs(static_cast<Eigen::Index>(inliers.size()));
  for (size_t row = 0; row < inliers.size(); ++row) {
    const CirclePoint& point = points[inliers[row]];
    system.row(static_cast<Eigen::Index>(row)) << 2.0 * point.x, 2.0 * point.y, 1.0;
    rhs(static_cast<Eigen::Index>(row)) = point.x * point.x + point.y * point.y;
  }
  const Eigen::Vector3d solution = system.colPivHouseholderQr().solve(rhs);
  center_x = solution.x();
  center_y = solution.y();
  const double radius_sq = solution.z() + center_x * center_x + center_y * center_y;
  if (!solution.allFinite() || radius_sq <= 1e-12) return false;
  radius = std::sqrt(radius_sq);
  return std::isfinite(radius);
}

std::vector<size_t> circleInliers(const std::vector<CirclePoint>& points, double center_x,
                                  double center_y, double radius, double threshold,
                                  double* residual_sum = nullptr) {
  std::vector<size_t> inliers;
  inliers.reserve(points.size());
  double sum = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    const double residual =
        std::abs(std::hypot(points[i].x - center_x, points[i].y - center_y) - radius);
    if (residual <= threshold) {
      inliers.push_back(i);
      sum += residual;
    }
  }
  if (residual_sum != nullptr) *residual_sum = sum;
  return inliers;
}

CylinderModel fitCylinderOnce(const PoleCloud& cloud, const std::vector<size_t>& indices,
                              const PoleExtractionConfig& config, uint32_t random_seed) {
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  CylinderModel model;
  model.point_count = indices.size();
  if (indices.size() < 3) return model;

  V3 mean = V3::Zero();
  for (size_t index : indices) {
    const PolePoint& point = cloud[index];
    mean += V3(point.x, point.y, point.z);
  }
  mean /= static_cast<double>(indices.size());

  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  for (size_t index : indices) {
    const PolePoint& point = cloud[index];
    const V3 offset = V3(point.x, point.y, point.z) - mean;
    covariance.noalias() += offset * offset.transpose();
  }
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covariance);
  if (eigensolver.info() != Eigen::Success) return model;
  model.axis = eigensolver.eigenvectors().col(2).normalized();
  if (model.axis.z() < 0.0) model.axis = -model.axis;
  if (!model.axis.allFinite()) return model;
  model.pca_valid = true;
  model.tilt_deg =
      std::acos(std::clamp(model.axis.z(), -1.0, 1.0)) * kRadiansToDegrees;

  const V3 reference =
      std::abs(model.axis.z()) < 0.9 ? V3::UnitZ() : V3::UnitX();
  const V3 basis_x = model.axis.cross(reference).normalized();
  const V3 basis_y = model.axis.cross(basis_x).normalized();
  std::vector<CirclePoint> projected;
  projected.reserve(indices.size());
  for (size_t index : indices) {
    const PolePoint& point = cloud[index];
    const V3 offset = V3(point.x, point.y, point.z) - mean;
    projected.push_back({offset.dot(basis_x), offset.dot(basis_y)});
  }

  std::mt19937 generator(random_seed);
  std::uniform_int_distribution<size_t> distribution(0, projected.size() - 1);
  std::vector<size_t> best_inliers;
  double best_residual_sum = std::numeric_limits<double>::infinity();
  double best_center_x = 0.0;
  double best_center_y = 0.0;
  double best_radius = 0.0;
  for (int iteration = 0; iteration < config.cylinder_ransac_iterations; ++iteration) {
    size_t first = distribution(generator);
    size_t second = distribution(generator);
    size_t third = distribution(generator);
    for (int retry = 0; retry < 8 && (second == first || third == first || third == second);
         ++retry) {
      second = distribution(generator);
      third = distribution(generator);
    }
    if (second == first || third == first || third == second) continue;
    double center_x = 0.0;
    double center_y = 0.0;
    double radius = 0.0;
    if (!circleFromThree(projected[first], projected[second], projected[third], center_x,
                         center_y, radius) ||
        radius < config.cylinder_min_radius || radius > config.cylinder_max_radius) {
      continue;
    }
    double residual_sum = 0.0;
    std::vector<size_t> inliers =
        circleInliers(projected, center_x, center_y, radius,
                      config.cylinder_inlier_threshold, &residual_sum);
    if (inliers.size() > best_inliers.size() ||
        (inliers.size() == best_inliers.size() && residual_sum < best_residual_sum)) {
      best_inliers = std::move(inliers);
      best_residual_sum = residual_sum;
      best_center_x = center_x;
      best_center_y = center_y;
      best_radius = radius;
    }
  }
  if (best_inliers.size() < 3) return model;

  // Refit twice: the first algebraic fit improves the RANSAC hypothesis, and
  // the second uses its updated consensus set.
  for (int refinement = 0; refinement < 2; ++refinement) {
    if (!leastSquaresCircle(projected, best_inliers, best_center_x, best_center_y,
                            best_radius)) {
      return model;
    }
    best_inliers = circleInliers(projected, best_center_x, best_center_y, best_radius,
                                 config.cylinder_inlier_threshold);
    if (best_inliers.size() < 3) return model;
  }

  double squared_error_sum = 0.0;
  for (const CirclePoint& point : projected) {
    const double residual =
        std::hypot(point.x - best_center_x, point.y - best_center_y) - best_radius;
    squared_error_sum += residual * residual;
  }
  model.radius = best_radius;
  model.rms = std::sqrt(squared_error_sum / static_cast<double>(projected.size()));
  model.inlier_ratio =
      static_cast<double>(best_inliers.size()) / static_cast<double>(projected.size());
  model.axis_point = mean + best_center_x * basis_x + best_center_y * basis_y;

  constexpr int kAngularBins = 36;
  std::array<bool, kAngularBins> occupied{};
  for (size_t index : best_inliers) {
    const double angle = std::atan2(projected[index].y - best_center_y,
                                    projected[index].x - best_center_x);
    int bin = static_cast<int>(std::floor((angle + 3.14159265358979323846) /
                                          (2.0 * 3.14159265358979323846) *
                                          kAngularBins));
    bin = std::clamp(bin, 0, kAngularBins - 1);
    occupied[static_cast<size_t>(bin)] = true;
  }
  model.angular_coverage = static_cast<double>(
      std::count(occupied.begin(), occupied.end(), true)) / kAngularBins;
  model.valid = model.radius >= config.cylinder_min_radius &&
                model.radius <= config.cylinder_max_radius &&
                model.rms <= config.cylinder_max_rms &&
                model.inlier_ratio >= config.cylinder_min_inlier_ratio &&
                model.angular_coverage >= config.cylinder_min_angular_coverage &&
                model.tilt_deg <= config.cylinder_max_axis_tilt_deg;
  return model;
}

bool fitCylinderAdaptive(const PoleCloud& cloud, const std::vector<size_t>& indices,
                         const PoleExtractionConfig& config, int depth, uint32_t random_seed,
                         std::vector<CylinderModel>& accepted) {
  const CylinderModel model = fitCylinderOnce(cloud, indices, config, random_seed);
  if (model.valid) {
    accepted.push_back(model);
    return true;
  }
  if (!model.pca_valid || depth >= config.cylinder_max_piecewise_depth ||
      indices.size() < 2U * static_cast<size_t>(config.cylinder_min_segment_points)) {
    return false;
  }

  std::vector<size_t> ordered = indices;
  std::sort(ordered.begin(), ordered.end(), [&cloud, &model](size_t lhs, size_t rhs) {
    const PolePoint& left = cloud[lhs];
    const PolePoint& right = cloud[rhs];
    return V3(left.x, left.y, left.z).dot(model.axis) <
           V3(right.x, right.y, right.z).dot(model.axis);
  });
  const size_t middle = ordered.size() / 2;
  std::vector<size_t> lower(ordered.begin(), ordered.begin() + middle);
  std::vector<size_t> upper(ordered.begin() + middle, ordered.end());
  if (lower.size() < static_cast<size_t>(config.cylinder_min_segment_points) ||
      upper.size() < static_cast<size_t>(config.cylinder_min_segment_points)) {
    return false;
  }

  const size_t rollback = accepted.size();
  const bool lower_valid = fitCylinderAdaptive(cloud, lower, config, depth + 1,
                                               random_seed + 0x9e3779b9U, accepted);
  const bool upper_valid = fitCylinderAdaptive(cloud, upper, config, depth + 1,
                                               random_seed + 0x85ebca6bU, accepted);
  if (!lower_valid || !upper_valid) {
    accepted.resize(rollback);
    return false;
  }
  return true;
}

ExtractionResult extract(const ExtractionTask& task, const PoleExtractionConfig& config) {
  ExtractionResult result;
  result.downsampled =
      voxelDownsample(accumulateFrames(task.frames, config), config.input_voxel_size);
  if (result.downsampled.empty()) return result;

  float global_z_min = std::numeric_limits<float>::infinity();
  for (const PolePoint& point : result.downsampled) {
    global_z_min = std::min(global_z_min, point.z);
  }

  std::unordered_map<Cell, GridStat, CellHash> grid;
  grid.reserve(result.downsampled.size() / 8 + 1);
  for (const PolePoint& point : result.downsampled) {
    GridStat& stat = grid[toCell(point, config.grid_resolution)];
    ++stat.count;
    stat.z_min = std::min(stat.z_min, point.z);
    stat.z_max = std::max(stat.z_max, point.z);
    stat.layers.insert(static_cast<int>(
        std::floor((static_cast<double>(point.z) - global_z_min) / config.layer_resolution)));
  }

  std::unordered_set<Cell, CellHash> candidate_cells;
  candidate_cells.reserve(grid.size());
  for (const auto& [cell, stat] : grid) {
    const double height_span = static_cast<double>(stat.z_max - stat.z_min);
    if (stat.count > config.min_points_per_cell && height_span > config.min_height_span &&
        longestContinuousRun(stat.layers) >= config.min_continuous_layers) {
      candidate_cells.insert(cell);
    }
  }
  if (candidate_cells.empty()) return result;

  const auto valid_cells =
      smallComponents(closeMask(candidate_cells), config.max_component_cells);
  if (valid_cells.empty()) return result;

  result.candidates.reserve(result.downsampled.size() / 20 + 1);
  for (const PolePoint& point : result.downsampled) {
    if (valid_cells.find(toCell(point, config.grid_resolution)) != valid_cells.end()) {
      result.candidates.push_back(point);
    }
  }
  if (result.candidates.empty()) return result;

  const auto clusters = euclideanClusters(result.candidates, config.cluster_tolerance);
  for (const std::vector<size_t>& indices : clusters) {
    if (indices.size() < static_cast<size_t>(config.min_cluster_points) ||
        indices.size() > static_cast<size_t>(config.max_cluster_points)) {
      continue;
    }
    double sum_x = 0.0;
    double sum_y = 0.0;
    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    for (size_t index : indices) {
      const PolePoint& point = result.candidates[index];
      sum_x += point.x;
      sum_y += point.y;
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
    }
    const double span_x = static_cast<double>(max_x - min_x);
    const double span_y = static_cast<double>(max_y - min_y);
    if (span_x > config.max_xy_span || span_y > config.max_xy_span) continue;

    const double coarse_center_x = sum_x / static_cast<double>(indices.size());
    const double coarse_center_y = sum_y / static_cast<double>(indices.size());
    double cluster_radius = 0.0;
    for (size_t index : indices) {
      const PolePoint& point = result.candidates[index];
      cluster_radius =
          std::max(cluster_radius,
                   std::hypot(static_cast<double>(point.x) - coarse_center_x,
                              static_cast<double>(point.y) - coarse_center_y));
    }

    std::vector<CylinderModel> cylinder_segments;
    if (config.cylinder_fit_enable) {
      const uint32_t seed = static_cast<uint32_t>(task.index * 2654435761ULL + indices.front());
      if (!fitCylinderAdaptive(result.candidates, indices, config, 0, seed,
                               cylinder_segments)) {
        continue;
      }
    } else {
      CylinderModel model;
      model.axis = V3::UnitZ();
      model.axis_point = V3(coarse_center_x, coarse_center_y, 0.0);
      model.radius = cluster_radius;
      model.rms = 0.0;
      model.inlier_ratio = 1.0;
      model.angular_coverage = 1.0;
      model.tilt_deg = 0.0;
      model.point_count = indices.size();
      model.pca_valid = true;
      model.valid = true;
      cylinder_segments.push_back(model);
    }

    const double target_z =
        nearestTrajectoryZ(coarse_center_x, coarse_center_y, task.frames) +
        config.center_z_offset;
    V3 weighted_center = V3::Zero();
    V3 weighted_axis = V3::Zero();
    double weighted_radius = 0.0;
    double weighted_squared_rms = 0.0;
    double weighted_inlier_ratio = 0.0;
    double weighted_angular_coverage = 0.0;
    size_t fitted_points = 0;
    bool usable_axis = true;
    for (const CylinderModel& model : cylinder_segments) {
      if (std::abs(model.axis.z()) < 1e-3) {
        usable_axis = false;
        break;
      }
      const double weight = static_cast<double>(model.point_count);
      const V3 center =
          model.axis_point + (target_z - model.axis_point.z()) / model.axis.z() * model.axis;
      weighted_center += weight * center;
      weighted_axis += weight * model.axis;
      weighted_radius += weight * model.radius;
      weighted_squared_rms += weight * model.rms * model.rms;
      weighted_inlier_ratio += weight * model.inlier_ratio;
      weighted_angular_coverage += weight * model.angular_coverage;
      fitted_points += model.point_count;
    }
    if (!usable_axis || fitted_points == 0 || weighted_axis.norm() < 1e-9) continue;

    const double weight = static_cast<double>(fitted_points);
    const V3 axis = weighted_axis.normalized();
    const V3 center = weighted_center / weight;
    constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
    double axial_min = std::numeric_limits<double>::infinity();
    double axial_max = -std::numeric_limits<double>::infinity();
    for (size_t index : indices) {
      const PolePoint& point = result.candidates[index];
      const double axial_position =
          (V3(point.x, point.y, point.z) - center).dot(axis);
      axial_min = std::min(axial_min, axial_position);
      axial_max = std::max(axial_max, axial_position);
    }

    Detection detection;
    detection.keyframe = task.index;
    detection.stamp = task.frames.back().stamp;
    detection.center = center;
    detection.axis = axis;
    detection.radius = weighted_radius / weight;
    detection.cluster_radius = cluster_radius;
    detection.fit_rms = std::sqrt(weighted_squared_rms / weight);
    detection.inlier_ratio = weighted_inlier_ratio / weight;
    detection.angular_coverage = weighted_angular_coverage / weight;
    detection.axis_tilt_deg =
        std::acos(std::clamp(axis.z(), -1.0, 1.0)) * kRadiansToDegrees;
    detection.axial_min = axial_min;
    detection.axial_max = axial_max;
    detection.cylinder_segments = cylinder_segments.size();
    detection.span_x = span_x;
    detection.span_y = span_y;
    detection.points = indices.size();
    result.detections.push_back(detection);
    for (size_t index : indices) result.refined.push_back(result.candidates[index]);
  }
  return result;
}

template <typename T>
bool loadPositive(const config_internal::MergedYaml& yaml, const char* key, const T& fallback,
                  T& output) {
  output = yaml.get<T>("pole_extraction", key, fallback);
  if (output <= T(0)) {
    LOG(E, "Config error: 'pole_extraction." << key << "' must be positive, got " << output
                                               << ".");
    return false;
  }
  return true;
}

}  // namespace

bool loadPoleExtractionConfig(const std::vector<std::string>& yaml_paths,
                              PoleExtractionConfig& config) {
  config_internal::MergedYaml yaml;
  for (const std::string& path : yaml_paths) {
    if (path.empty()) continue;
    try {
      yaml.add(YAML::LoadFile(path));
    } catch (const std::exception& error) {
      LOG(E, "Failed to load pole extraction config from '" << path << "': " << error.what());
      return false;
    }
  }

  config.enable = yaml.get<bool>("pole_extraction", "enable", config.enable);
  config.process_partial_keyframe = yaml.get<bool>(
      "pole_extraction", "process_partial_keyframe", config.process_partial_keyframe);
  config.use_roi = yaml.get<bool>("pole_extraction", "use_roi", config.use_roi);
  config.cylinder_fit_enable = yaml.get<bool>(
      "pole_extraction", "cylinder_fit_enable", config.cylinder_fit_enable);
  config.save_keyframe_clouds = yaml.get<bool>(
      "pole_extraction", "save_keyframe_clouds", config.save_keyframe_clouds);

  int frames_per_keyframe = static_cast<int>(config.frames_per_keyframe);
  int min_partial_frames = static_cast<int>(config.min_partial_frames);
  int max_queue = static_cast<int>(config.max_queue);
  if (!loadPositive(yaml, "frames_per_keyframe", frames_per_keyframe, frames_per_keyframe) ||
      !loadPositive(yaml, "min_partial_frames", min_partial_frames, min_partial_frames) ||
      !loadPositive(yaml, "max_queue", max_queue, max_queue) ||
      !loadPositive(yaml, "input_voxel_resolution_m", config.input_voxel_size,
                    config.input_voxel_size) ||
      !loadPositive(yaml, "grid_resolution_m", config.grid_resolution,
                    config.grid_resolution) ||
      !loadPositive(yaml, "min_points_per_cell", config.min_points_per_cell,
                    config.min_points_per_cell) ||
      !loadPositive(yaml, "min_height_span_m", config.min_height_span,
                    config.min_height_span) ||
      !loadPositive(yaml, "layer_resolution_m", config.layer_resolution,
                    config.layer_resolution) ||
      !loadPositive(yaml, "min_continuous_layers", config.min_continuous_layers,
                    config.min_continuous_layers) ||
      !loadPositive(yaml, "max_component_cells", config.max_component_cells,
                    config.max_component_cells) ||
      !loadPositive(yaml, "cluster_tolerance_m", config.cluster_tolerance,
                    config.cluster_tolerance) ||
      !loadPositive(yaml, "max_xy_span_m", config.max_xy_span, config.max_xy_span) ||
      !loadPositive(yaml, "min_cluster_points", config.min_cluster_points,
                    config.min_cluster_points) ||
      !loadPositive(yaml, "max_cluster_points", config.max_cluster_points,
                    config.max_cluster_points) ||
      !loadPositive(yaml, "cylinder_ransac_iterations", config.cylinder_ransac_iterations,
                    config.cylinder_ransac_iterations) ||
      !loadPositive(yaml, "cylinder_inlier_threshold_m",
                    config.cylinder_inlier_threshold,
                    config.cylinder_inlier_threshold) ||
      !loadPositive(yaml, "cylinder_min_inlier_ratio",
                    config.cylinder_min_inlier_ratio,
                    config.cylinder_min_inlier_ratio) ||
      !loadPositive(yaml, "cylinder_max_rms_m", config.cylinder_max_rms,
                    config.cylinder_max_rms) ||
      !loadPositive(yaml, "cylinder_min_radius_m", config.cylinder_min_radius,
                    config.cylinder_min_radius) ||
      !loadPositive(yaml, "cylinder_max_radius_m", config.cylinder_max_radius,
                    config.cylinder_max_radius) ||
      !loadPositive(yaml, "cylinder_max_axis_tilt_deg",
                    config.cylinder_max_axis_tilt_deg,
                    config.cylinder_max_axis_tilt_deg) ||
      !loadPositive(yaml, "cylinder_min_angular_coverage",
                    config.cylinder_min_angular_coverage,
                    config.cylinder_min_angular_coverage) ||
      !loadPositive(yaml, "cylinder_min_segment_points",
                    config.cylinder_min_segment_points,
                    config.cylinder_min_segment_points) ||
      !loadPositive(yaml, "landmark_merge_radius_m", config.landmark_merge_radius,
                    config.landmark_merge_radius) ||
      !loadPositive(yaml, "landmark_max_axis_angle_deg",
                    config.landmark_max_axis_angle_deg,
                    config.landmark_max_axis_angle_deg) ||
      !loadPositive(yaml, "landmark_max_radius_difference_m",
                    config.landmark_max_radius_difference,
                    config.landmark_max_radius_difference)) {
    return false;
  }
  config.frames_per_keyframe = static_cast<size_t>(frames_per_keyframe);
  config.min_partial_frames = static_cast<size_t>(min_partial_frames);
  config.max_queue = static_cast<size_t>(max_queue);
  config.cylinder_max_piecewise_depth = yaml.get<int>(
      "pole_extraction", "cylinder_max_piecewise_depth",
      config.cylinder_max_piecewise_depth);

  config.roi_y_min = yaml.get<double>("pole_extraction", "roi_y_min_m", config.roi_y_min);
  config.roi_y_max = yaml.get<double>("pole_extraction", "roi_y_max_m", config.roi_y_max);
  config.roi_z_min = yaml.get<double>("pole_extraction", "roi_z_min_m", config.roi_z_min);
  config.roi_z_max = yaml.get<double>("pole_extraction", "roi_z_max_m", config.roi_z_max);
  config.center_z_offset =
      yaml.get<double>("pole_extraction", "center_z_offset_m", config.center_z_offset);
  config.output_directory = yaml.get<std::string>(
      "pole_extraction", "output_directory", config.output_directory);

  if (config.min_partial_frames > config.frames_per_keyframe) {
    LOG(E, "Config error: pole_extraction.min_partial_frames cannot exceed "
               "frames_per_keyframe.");
    return false;
  }
  if (config.roi_y_min >= config.roi_y_max || config.roi_z_min >= config.roi_z_max) {
    LOG(E, "Config error: pole extraction ROI minimums must be smaller than maximums.");
    return false;
  }
  if (config.min_cluster_points > config.max_cluster_points) {
    LOG(E, "Config error: pole_extraction.min_cluster_points cannot exceed "
               "max_cluster_points.");
    return false;
  }
  if (config.cylinder_max_piecewise_depth < 0 ||
      config.cylinder_min_radius >= config.cylinder_max_radius ||
      config.cylinder_min_inlier_ratio > 1.0 ||
      config.cylinder_min_angular_coverage > 1.0 ||
      config.cylinder_max_axis_tilt_deg > 90.0 ||
      config.landmark_max_axis_angle_deg > 90.0) {
    LOG(E, "Config error: invalid pole cylinder fitting range or ratio.");
    return false;
  }

  if (config.enable) {
    LOG(I, "Pole extraction enabled: " << config.frames_per_keyframe
                                        << " scans/keyframe, grid "
                                        << config.grid_resolution << " m, output '"
                                        << config.output_directory << "', cylinder fit "
                                        << (config.cylinder_fit_enable ? "on" : "off") << ".");
  }
  return true;
}

class PoleExtractor::Impl {
 public:
  Impl(ros::NodeHandle& nh, const std::shared_ptr<Pipeline>& pipeline,
       PoleExtractionConfig config, const std::string& topic_prefix)
      : config_(std::move(config)), map_frame_(pipeline->config().map_frame) {
    centers_pub_ =
        nh.advertise<sensor_msgs::PointCloud2>(topic_prefix + "/poles/centers", 2, true);
    points_pub_ =
        nh.advertise<sensor_msgs::PointCloud2>(topic_prefix + "/poles/points", 2, true);
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_prefix + "/poles/map", 1, true);
    markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>(
        topic_prefix + "/poles/markers", 1, true);
    worker_ = std::thread(&Impl::workerLoop, this);
    pipeline->addFrameObserver(
        [this](uint64_t stamp, const Transform& pose, const Pointcloud& cloud) {
          addFrame(stamp, pose, cloud);
        });
  }

  ~Impl() { finish(); }

  void addFrame(uint64_t stamp, const Transform& pose, const Pointcloud& cloud) {
    if (!accepting_.load(std::memory_order_acquire)) return;
    ExtractionFrame frame;
    frame.stamp = stamp;
    frame.pose = pose;
    frame.cloud = cloud;

    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (!accepting_.load(std::memory_order_relaxed)) return;
    ++received_frames_;
    current_frames_.push_back(std::move(frame));
    if (current_frames_.size() < config_.frames_per_keyframe) return;
    queueCurrentKeyframeLocked();
    queue_cv_.notify_one();
  }

  void finish() {
    bool expected = true;
    if (!accepting_.compare_exchange_strong(expected, false)) {
      if (worker_.joinable()) worker_.join();
      return;
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      if (config_.process_partial_keyframe &&
          current_frames_.size() >= config_.min_partial_frames) {
        queueCurrentKeyframeLocked();
      } else {
        current_frames_.clear();
      }
      stop_ = true;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

  PoleExtractionStats stats() const {
    PoleExtractionStats output;
    output.received_frames = received_frames_.load();
    output.processed_keyframes = processed_keyframes_.load();
    output.dropped_keyframes = dropped_keyframes_.load();
    output.detections = detection_count_.load();
    std::lock_guard<std::mutex> lock(results_mutex_);
    output.landmarks = landmarks_.size();
    return output;
  }

  bool saveResults(std::string* message) const {
    namespace fs = std::filesystem;
    std::vector<Detection> detections;
    std::vector<Landmark> landmarks;
    {
      std::lock_guard<std::mutex> lock(results_mutex_);
      detections = detections_;
      landmarks = landmarks_;
    }
    if (config_.output_directory.empty()) {
      if (message) *message = "Pole output directory is empty; results were not saved.";
      return false;
    }

    std::error_code error;
    fs::create_directories(config_.output_directory, error);
    if (error) {
      if (message) *message = "Failed to create pole output directory: " + error.message();
      return false;
    }

    std::sort(landmarks.begin(), landmarks.end(),
              [](const Landmark& lhs, const Landmark& rhs) { return lhs.id < rhs.id; });

    const fs::path directory(config_.output_directory);
    std::ofstream landmark_file(directory / "landmarks.txt");
    std::ofstream detection_file(directory / "detections.txt");
    std::ofstream summary_file(directory / "summary.yaml");
    if (!landmark_file || !detection_file || !summary_file) {
      if (message) *message = "Failed to open one or more pole result files.";
      return false;
    }

    landmark_file
        << "ID X Y Z AxisX AxisY AxisZ AxialMin AxialMax Observations TotalPoints Radius FitRMS\n"
        << std::fixed << std::setprecision(6);
    for (const Landmark& landmark : landmarks) {
      landmark_file << landmark.id << " " << landmark.center.x() << " "
                    << landmark.center.y() << " "
                    << landmark.center.z() << " " << landmark.axis.x() << " "
                    << landmark.axis.y() << " " << landmark.axis.z() << " "
                    << landmark.axial_min << " " << landmark.axial_max << " "
                    << landmark.observations << " " << landmark.total_points << " "
                    << landmark.radius << " " << landmark.fit_rms << "\n";
    }

    detection_file
        << "Keyframe Timestamp X Y Z AxisX AxisY AxisZ CylinderRadius FitRMS "
           "InlierRatio AngularCoverage AxisTiltDeg Segments ClusterRadius SpanX SpanY "
           "AxialMin AxialMax TotalPoints\n"
        << std::fixed << std::setprecision(9);
    for (const Detection& detection : detections) {
      detection_file << detection.keyframe << " " << nsToS(detection.stamp) << " "
                     << detection.center.x() << " " << detection.center.y() << " "
                     << detection.center.z() << " " << detection.axis.x() << " "
                     << detection.axis.y() << " " << detection.axis.z() << " "
                     << detection.radius << " " << detection.fit_rms << " "
                     << detection.inlier_ratio << " " << detection.angular_coverage << " "
                     << detection.axis_tilt_deg << " " << detection.cylinder_segments << " "
                     << detection.cluster_radius << " " << detection.span_x << " "
                     << detection.span_y << " " << detection.axial_min << " "
                     << detection.axial_max << " " << detection.points << "\n";
    }

    const PoleExtractionStats snapshot = stats();
    summary_file << "received_frames: " << snapshot.received_frames << "\n"
                 << "processed_keyframes: " << snapshot.processed_keyframes << "\n"
                 << "dropped_keyframes: " << snapshot.dropped_keyframes << "\n"
                 << "detections: " << snapshot.detections << "\n"
                 << "landmarks: " << snapshot.landmarks << "\n";
    if (!landmark_file || !detection_file || !summary_file) {
      if (message) *message = "Failed while writing pole text results.";
      return false;
    }

    if (!writeLandmarkPcd(directory / "landmarks.pcd", landmarks)) {
      if (message) *message = "Failed to save pole landmark PCD.";
      return false;
    }

    if (message) {
      *message = "Saved " + std::to_string(landmarks.size()) + " pole landmarks (" +
                 std::to_string(detections.size()) + " detections) to '" +
                 directory.string() + "'.";
    }
    return true;
  }

 private:
  void queueCurrentKeyframeLocked() {
    ExtractionTask task;
    task.index = next_keyframe_index_++;
    task.frames = std::move(current_frames_);
    current_frames_.clear();
    if (queue_.size() >= config_.max_queue) {
      queue_.pop_front();
      ++dropped_keyframes_;
    }
    queue_.push_back(std::move(task));
  }

  void workerLoop() {
    while (true) {
      ExtractionTask task;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return stop_ || !queue_.empty(); });
        if (queue_.empty() && stop_) return;
        task = std::move(queue_.front());
        queue_.pop_front();
      }

      const auto start = std::chrono::steady_clock::now();
      ExtractionResult result = extract(task, config_);
      integrateDetections(result.detections);
      publish(task.frames.back().stamp, result);
      saveKeyframeCloud(task.index, result.refined);
      ++processed_keyframes_;
      detection_count_.fetch_add(result.detections.size());
      const double seconds =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
      LOG(I, "Pole keyframe " << task.index << ": " << task.frames.size() << " scans, "
                               << result.downsampled.size() << " ROI/downsampled points, "
                               << result.candidates.size() << " candidates, "
                               << result.detections.size() << " poles in " << std::fixed
                               << std::setprecision(3) << seconds << " s.");
    }
  }

  void integrateDetections(const std::vector<Detection>& detections) {
    std::lock_guard<std::mutex> lock(results_mutex_);
    detections_.insert(detections_.end(), detections.begin(), detections.end());
    const double merge_distance_sq =
        config_.landmark_merge_radius * config_.landmark_merge_radius;
    constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
    const double minimum_axis_dot =
        std::cos(config_.landmark_max_axis_angle_deg * kDegreesToRadians);
    for (const Detection& detection : detections) {
      Landmark* match = nullptr;
      double best_distance_sq = merge_distance_sq;
      for (Landmark& landmark : landmarks_) {
        const double distance_sq =
            (landmark.center - detection.center).head<2>().squaredNorm();
        const double axis_dot = std::abs(landmark.axis.dot(detection.axis));
        const double radius_difference = std::abs(landmark.radius - detection.radius);
        if (distance_sq <= best_distance_sq && axis_dot >= minimum_axis_dot &&
            radius_difference <= config_.landmark_max_radius_difference) {
          match = &landmark;
          best_distance_sq = distance_sq;
        }
      }
      if (match == nullptr) {
        Landmark landmark;
        landmark.id = next_landmark_id_++;
        landmark.center = detection.center;
        landmark.axis = detection.axis;
        landmark.radius = detection.radius;
        landmark.fit_rms = detection.fit_rms;
        landmark.axial_min = detection.axial_min;
        landmark.axial_max = detection.axial_max;
        landmark.observations = 1;
        landmark.total_points = detection.points;
        landmarks_.push_back(landmark);
      } else {
        const double count = static_cast<double>(match->observations);
        match->center = (match->center * count + detection.center) / (count + 1.0);
        V3 aligned_axis = detection.axis;
        if (match->axis.dot(aligned_axis) < 0.0) aligned_axis = -aligned_axis;
        match->axis = (match->axis * count + aligned_axis).normalized();
        match->radius = (match->radius * count + detection.radius) / (count + 1.0);
        match->fit_rms = (match->fit_rms * count + detection.fit_rms) / (count + 1.0);
        match->axial_min =
            (match->axial_min * count + detection.axial_min) / (count + 1.0);
        match->axial_max =
            (match->axial_max * count + detection.axial_max) / (count + 1.0);
        ++match->observations;
        match->total_points += detection.points;
      }
    }
  }

  void publishCloud(const Pointcloud& cloud, uint64_t stamp, const ros::Publisher& publisher) {
    sensor_msgs::PointCloud2 message;
    pointCloudToMsg(cloud, message);
    message.header.stamp.fromNSec(stamp);
    message.header.frame_id = map_frame_;
    publisher.publish(message);
  }

  void publishMarkers(uint64_t stamp, const std::vector<Landmark>& landmarks) {
    visualization_msgs::MarkerArray array;
    visualization_msgs::Marker clear;
    clear.header.frame_id = map_frame_;
    clear.header.stamp.fromNSec(stamp);
    clear.action = visualization_msgs::Marker::DELETEALL;
    array.markers.push_back(clear);

    auto setCommon = [this, stamp](visualization_msgs::Marker& marker,
                                  const std::string& marker_namespace, int id, int type) {
      marker.header.frame_id = map_frame_;
      marker.header.stamp.fromNSec(stamp);
      marker.ns = marker_namespace;
      marker.id = id;
      marker.type = type;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
    };
    auto toPoint = [](const V3& point) {
      geometry_msgs::Point output;
      output.x = point.x();
      output.y = point.y();
      output.z = point.z();
      return output;
    };

    for (size_t index = 0; index < landmarks.size(); ++index) {
      const Landmark& landmark = landmarks[index];
      const int id = static_cast<int>(landmark.id);
      const double height = std::max(0.20, landmark.axial_max - landmark.axial_min);
      const V3 lower = landmark.center + landmark.axial_min * landmark.axis;
      const V3 upper = landmark.center + landmark.axial_max * landmark.axis;
      const V3 middle = 0.5 * (lower + upper);

      visualization_msgs::Marker cylinder;
      setCommon(cylinder, "pole_cylinder", id, visualization_msgs::Marker::CYLINDER);
      cylinder.pose.position = toPoint(middle);
      Eigen::Quaterniond orientation;
      orientation.setFromTwoVectors(V3::UnitZ(), landmark.axis);
      cylinder.pose.orientation.x = orientation.x();
      cylinder.pose.orientation.y = orientation.y();
      cylinder.pose.orientation.z = orientation.z();
      cylinder.pose.orientation.w = orientation.w();
      cylinder.scale.x = std::max(0.10, 2.0 * landmark.radius);
      cylinder.scale.y = cylinder.scale.x;
      cylinder.scale.z = height;
      cylinder.color.r = 0.10F;
      cylinder.color.g = 0.95F;
      cylinder.color.b = 0.25F;
      cylinder.color.a = 0.28F;
      array.markers.push_back(cylinder);

      visualization_msgs::Marker axis;
      setCommon(axis, "pole_axis", id, visualization_msgs::Marker::ARROW);
      axis.points.push_back(toPoint(lower));
      axis.points.push_back(toPoint(upper));
      axis.scale.x = 0.05;
      axis.scale.y = 0.13;
      axis.scale.z = 0.22;
      axis.color.r = 1.0F;
      axis.color.g = 0.82F;
      axis.color.b = 0.05F;
      axis.color.a = 1.0F;
      array.markers.push_back(axis);

      visualization_msgs::Marker base;
      setCommon(base, "pole_base", id, visualization_msgs::Marker::SPHERE);
      base.pose.position = toPoint(landmark.center);
      base.scale.x = 0.35;
      base.scale.y = 0.35;
      base.scale.z = 0.35;
      base.color.r = 1.0F;
      base.color.g = 0.20F;
      base.color.b = 0.05F;
      base.color.a = 1.0F;
      array.markers.push_back(base);

      visualization_msgs::Marker text_marker;
      setCommon(text_marker, "pole_id", id, visualization_msgs::Marker::TEXT_VIEW_FACING);
      text_marker.pose.position = toPoint(upper + V3(0.0, 0.0, 0.55));
      text_marker.scale.z = 0.48;
      text_marker.color.r = 1.0F;
      text_marker.color.g = 1.0F;
      text_marker.color.b = 1.0F;
      text_marker.color.a = 1.0F;
      std::ostringstream label;
      label << "Pole " << landmark.id << "  obs=" << landmark.observations << "\nr="
            << std::fixed << std::setprecision(2) << landmark.radius << " m  rms="
            << landmark.fit_rms;
      text_marker.text = label.str();
      array.markers.push_back(text_marker);
    }
    markers_pub_.publish(array);
  }

  void publish(uint64_t stamp, const ExtractionResult& result) {
    publishCloud(centersToCloud(result.detections), stamp, centers_pub_);
    publishCloud(toBievrCloud(result.refined), stamp, points_pub_);
    std::vector<Landmark> landmarks;
    {
      std::lock_guard<std::mutex> lock(results_mutex_);
      landmarks = landmarks_;
    }
    publishCloud(landmarksToCloud(landmarks), stamp, map_pub_);
    publishMarkers(stamp, landmarks);
  }

  void saveKeyframeCloud(size_t index, const PoleCloud& cloud) const {
    if (!config_.save_keyframe_clouds || cloud.empty() || config_.output_directory.empty()) {
      return;
    }
    std::error_code error;
    std::filesystem::create_directories(config_.output_directory, error);
    if (error) {
      LOG(W, "Cannot create pole keyframe output directory: " << error.message());
      return;
    }
    std::ostringstream name;
    name << "keyframe_" << std::setw(6) << std::setfill('0') << index << "_poles.pcd";
    const std::filesystem::path path =
        std::filesystem::path(config_.output_directory) / name.str();
    if (!writeXYZPcd(path, cloud)) {
      LOG(W, "Failed to save extracted pole cloud to '" << path.string() << "'.");
    }
  }

  PoleExtractionConfig config_;
  std::string map_frame_;
  ros::Publisher centers_pub_;
  ros::Publisher points_pub_;
  ros::Publisher map_pub_;
  ros::Publisher markers_pub_;

  std::atomic<bool> accepting_{true};
  bool stop_ = false;
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::vector<ExtractionFrame> current_frames_;
  std::deque<ExtractionTask> queue_;
  size_t next_keyframe_index_ = 0;
  std::thread worker_;

  mutable std::mutex results_mutex_;
  std::vector<Detection> detections_;
  std::vector<Landmark> landmarks_;
  size_t next_landmark_id_ = 0;

  std::atomic<size_t> received_frames_{0};
  std::atomic<size_t> processed_keyframes_{0};
  std::atomic<size_t> dropped_keyframes_{0};
  std::atomic<size_t> detection_count_{0};
};

PoleExtractor::PoleExtractor(ros::NodeHandle& nh, const std::shared_ptr<Pipeline>& pipeline,
                             PoleExtractionConfig config, const std::string& topic_prefix)
    : impl_(new Impl(nh, pipeline, std::move(config), topic_prefix)) {}

PoleExtractor::~PoleExtractor() = default;

void PoleExtractor::finish() { impl_->finish(); }

bool PoleExtractor::saveResults(std::string* message) const { return impl_->saveResults(message); }

PoleExtractionStats PoleExtractor::stats() const { return impl_->stats(); }

}  // namespace bievr
