#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_pgo/gnss_optimizer.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/NavSatFix.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Options {
  std::string bag;
  std::string lio_trajectory;
  std::string output_directory = "Log/gnss_optimization";
  std::string gnss_topic = "/gnss";
};

bool commandLineValue(const std::vector<std::string>& args, const std::string& key,
                      std::string& value) {
  for (size_t index = 0; index < args.size(); ++index) {
    if (args[index] == key && index + 1 < args.size()) {
      value = args[index + 1];
      return true;
    }
    const std::string prefix = key + "=";
    if (args[index].rfind(prefix, 0) == 0) {
      value = args[index].substr(prefix.size());
      return true;
    }
  }
  return false;
}

uint64_t secondsToNanoseconds(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0.0) return 0;
  return static_cast<uint64_t>(std::llround(seconds * 1.0e9));
}

bool readTum(const std::string& path,
             std::vector<bievr::GnssTrajectoryOptimizer::StampedPose>& poses,
             std::string& message) {
  std::ifstream input(path);
  if (!input) {
    message = "Cannot open LIO TUM trajectory '" + path + "'.";
    return false;
  }
  poses.clear();
  std::string line;
  size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    if (line.empty() || line.front() == '#') continue;
    std::istringstream stream(line);
    double stamp = 0.0;
    bievr::V3 translation;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (!(stream >> stamp >> translation.x() >> translation.y() >> translation.z() >>
          qx >> qy >> qz >> qw)) {
      message = "Malformed TUM row " + std::to_string(line_number) + " in '" + path + "'.";
      return false;
    }
    bievr::Quaternion quaternion(qw, qx, qy, qz);
    if (!translation.allFinite() || !quaternion.coeffs().allFinite() ||
        quaternion.norm() < 1.0e-9) continue;
    quaternion.normalize();
    poses.push_back({secondsToNanoseconds(stamp), bievr::Transform(quaternion, translation)});
  }
  std::sort(poses.begin(), poses.end(), [](const auto& a, const auto& b) {
    return a.stamp < b.stamp;
  });
  poses.erase(std::unique(poses.begin(), poses.end(), [](const auto& a, const auto& b) {
                return a.stamp == b.stamp;
              }),
              poses.end());
  if (poses.size() < 2) {
    message = "LIO TUM trajectory contains fewer than two valid poses.";
    return false;
  }
  return true;
}

class LocalEnu {
 public:
  LocalEnu(double latitude_deg, double longitude_deg, double altitude_m)
      : latitude_(latitude_deg * kPi / 180.0),
        longitude_(longitude_deg * kPi / 180.0), origin_(ecef(latitude_deg, longitude_deg,
                                                               altitude_m)) {
    rotation_ << -std::sin(longitude_), std::cos(longitude_), 0.0,
        -std::sin(latitude_) * std::cos(longitude_),
        -std::sin(latitude_) * std::sin(longitude_), std::cos(latitude_),
        std::cos(latitude_) * std::cos(longitude_),
        std::cos(latitude_) * std::sin(longitude_), std::sin(latitude_);
  }

  bievr::V3 convert(double latitude_deg, double longitude_deg, double altitude_m) const {
    return rotation_ * (ecef(latitude_deg, longitude_deg, altitude_m) - origin_);
  }

 private:
  static constexpr double kPi = 3.14159265358979323846;

  static bievr::V3 ecef(double latitude_deg, double longitude_deg, double altitude_m) {
    constexpr double semi_major_axis = 6378137.0;
    constexpr double eccentricity_squared = 6.6943799901413165e-3;
    const double latitude = latitude_deg * kPi / 180.0;
    const double longitude = longitude_deg * kPi / 180.0;
    const double sin_latitude = std::sin(latitude);
    const double radius = semi_major_axis /
                          std::sqrt(1.0 - eccentricity_squared * sin_latitude * sin_latitude);
    return bievr::V3((radius + altitude_m) * std::cos(latitude) * std::cos(longitude),
                     (radius + altitude_m) * std::cos(latitude) * std::sin(longitude),
                     (radius * (1.0 - eccentricity_squared) + altitude_m) * sin_latitude);
  }

  double latitude_;
  double longitude_;
  bievr::V3 origin_;
  bievr::M3 rotation_;
};

bool readGnss(const std::string& bag_path, const std::string& topic,
              std::vector<bievr::GnssTrajectoryOptimizer::PositionMeasurement>& measurements,
              bievr::V3& origin_lla, std::string& message) {
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (const std::exception& error) {
    message = std::string("Cannot open bag: ") + error.what();
    return false;
  }
  std::vector<sensor_msgs::NavSatFix::ConstPtr> fixes;
  for (const rosbag::MessageInstance& instance :
       rosbag::View(bag, rosbag::TopicQuery({topic}))) {
    const auto fix = instance.instantiate<sensor_msgs::NavSatFix>();
    if (!fix || fix->status.status < sensor_msgs::NavSatStatus::STATUS_FIX ||
        !std::isfinite(fix->latitude) || !std::isfinite(fix->longitude) ||
        !std::isfinite(fix->altitude) || std::abs(fix->latitude) > 90.0 ||
        std::abs(fix->longitude) > 180.0) {
      continue;
    }
    fixes.push_back(fix);
  }
  bag.close();
  if (fixes.size() < 3) {
    message = "Topic '" + topic + "' contains fewer than three valid NavSatFix messages.";
    return false;
  }
  origin_lla = bievr::V3(fixes.front()->latitude, fixes.front()->longitude,
                         fixes.front()->altitude);
  const LocalEnu converter(origin_lla.x(), origin_lla.y(), origin_lla.z());
  measurements.clear();
  measurements.reserve(fixes.size());
  for (const auto& fix : fixes) {
    bievr::V3 variance(1.0, 1.0, 4.0);
    if (fix->position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
      variance = bievr::V3(fix->position_covariance[0], fix->position_covariance[4],
                           fix->position_covariance[8]);
    }
    if (!variance.allFinite() || (variance.array() < 0.0).any()) {
      variance = bievr::V3(1.0, 1.0, 4.0);
    }
    measurements.push_back({fix->header.stamp.toNSec(),
                            converter.convert(fix->latitude, fix->longitude, fix->altitude),
                            variance});
  }
  return true;
}

bool writeOutputs(
    const Options& options, const bievr::V3& origin_lla,
    const std::vector<bievr::GnssTrajectoryOptimizer::StampedPose>& raw,
    const std::vector<bievr::GnssTrajectoryOptimizer::StampedPose>& optimized,
    const bievr::GnssTrajectoryOptimizer::Stats& stats, std::string& message) {
  namespace fs = std::filesystem;
  std::error_code error;
  fs::create_directories(options.output_directory, error);
  if (error) {
    message = "Cannot create output directory: " + error.message();
    return false;
  }
  const fs::path directory(options.output_directory);
  std::ofstream tum(directory / "gnss_optimized_tum.txt");
  std::ofstream summary(directory / "summary.yaml");
  std::ofstream seed(directory / "initial_correction.yaml");
  if (!tum || !summary || !seed) {
    message = "Cannot open one or more GNSS optimization output files.";
    return false;
  }
  tum << std::fixed << std::setprecision(9);
  for (const auto& stamped : optimized) {
    const bievr::V3& p = stamped.pose.translation();
    const bievr::Quaternion q(stamped.pose.rotation());
    tum << static_cast<double>(stamped.stamp) * 1.0e-9 << " " << p.x() << " " << p.y()
        << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
        << q.w() << "\n";
  }
  const bievr::Transform T_enu_odom =
      optimized.front().pose * raw.front().pose.inverse();
  const bievr::V3 translation = T_enu_odom.translation();
  const bievr::Quaternion correction_orientation(T_enu_odom.rotation());
  const double yaw_deg =
      std::atan2(T_enu_odom.rotation()(1, 0), T_enu_odom.rotation()(0, 0)) * 180.0 /
      3.14159265358979323846;
  seed << std::fixed << std::setprecision(9)
       << "# Pass this file as the last --params_file in the GNSS-free run.\n"
       << "pole_extraction:\n"
       << "  localization_use_initial_guess: true\n"
       << "  localization_initial_translation: [" << translation.x() << ", "
       << translation.y() << ", " << translation.z() << "]\n"
       << "  localization_initial_orientation_xyzw: [" << correction_orientation.x()
       << ", " << correction_orientation.y() << ", " << correction_orientation.z()
       << ", " << correction_orientation.w() << "]\n"
       << "  localization_initial_yaw_deg: " << yaw_deg << "\n";
  summary << std::fixed << std::setprecision(9)
          << "bag: \"" << options.bag << "\"\n"
          << "gnss_topic: \"" << options.gnss_topic << "\"\n"
          << "lio_trajectory: \"" << options.lio_trajectory << "\"\n"
          << "origin_lla: [" << origin_lla.x() << ", " << origin_lla.y() << ", "
          << origin_lla.z() << "]\n"
          << "input_poses: " << stats.input_poses << "\n"
          << "input_measurements: " << stats.input_measurements << "\n"
          << "used_measurements: " << stats.used_measurements << "\n"
          << "rejected_time: " << stats.rejected_time << "\n"
          << "rejected_outlier: " << stats.rejected_outlier << "\n"
          << "initial_horizontal_rmse_m: " << stats.initial_horizontal_rmse_m << "\n"
          << "optimized_horizontal_rmse_m: " << stats.optimized_horizontal_rmse_m << "\n"
          << "initial_3d_rmse_m: " << stats.initial_3d_rmse_m << "\n"
          << "optimized_3d_rmse_m: " << stats.optimized_3d_rmse_m << "\n"
          << "initial_graph_error: " << stats.initial_error << "\n"
          << "final_graph_error: " << stats.final_error << "\n"
          << "seed_translation: [" << translation.x() << ", " << translation.y() << ", "
          << translation.z() << "]\n"
          << "seed_orientation_xyzw: [" << correction_orientation.x() << ", "
          << correction_orientation.y() << ", " << correction_orientation.z() << ", "
          << correction_orientation.w() << "]\n"
          << "seed_yaw_deg: " << yaw_deg << "\n";
  if (!tum || !summary || !seed) {
    message = "Failed while writing GNSS optimization outputs.";
    return false;
  }
  message = "Saved GNSS-optimized trajectory and pole-localization seed to '" +
            options.output_directory + "'.";
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "bievr_gnss_optimize_node");
  Options options;
  bievr::GnssTrajectoryOptimizer::Config config;
  const std::vector<std::string> args(argv, argv + argc);
  commandLineValue(args, "--bag", options.bag);
  commandLineValue(args, "--lio_trajectory", options.lio_trajectory);
  commandLineValue(args, "--output_directory", options.output_directory);
  commandLineValue(args, "--gnss_topic", options.gnss_topic);
  // A roslaunch invocation has a master and can configure private parameters.
  // Direct offline CLI use intentionally does not require roscore.
  if ((options.bag.empty() || options.lio_trajectory.empty()) && ros::master::check()) {
    ros::NodeHandle private_node("~");
    private_node.param("bag", options.bag, options.bag);
    private_node.param("lio_trajectory", options.lio_trajectory, options.lio_trajectory);
    private_node.param("output_directory", options.output_directory, options.output_directory);
    private_node.param("gnss_topic", options.gnss_topic, options.gnss_topic);
    private_node.param("max_time_gap_s", config.max_time_gap_s, config.max_time_gap_s);
    private_node.param("horizontal_sigma_floor_m", config.horizontal_sigma_floor_m,
                       config.horizontal_sigma_floor_m);
    private_node.param("vertical_sigma_floor_m", config.vertical_sigma_floor_m,
                       config.vertical_sigma_floor_m);
    private_node.param("gnss_huber_delta", config.gnss_huber_delta, config.gnss_huber_delta);
    private_node.param("initial_outlier_gate_m", config.initial_outlier_gate_m,
                       config.initial_outlier_gate_m);
    private_node.param("odom_rotation_sigma_deg_per_sqrt_m",
                       config.odom_rotation_sigma_deg_per_sqrt_m,
                       config.odom_rotation_sigma_deg_per_sqrt_m);
    private_node.param("odom_translation_sigma_m_per_sqrt_m",
                       config.odom_translation_sigma_m_per_sqrt_m,
                       config.odom_translation_sigma_m_per_sqrt_m);
    private_node.param("minimum_edge_distance_m", config.minimum_edge_distance_m,
                       config.minimum_edge_distance_m);
    private_node.param("max_iterations", config.max_iterations, config.max_iterations);
  }
  if (options.bag.empty() || options.lio_trajectory.empty()) {
    LOG(E, "Usage: gnss_optimize --bag BAG --lio_trajectory TRAJECTORY "
           "[--output_directory DIR] [--gnss_topic /gnss]");
    return 2;
  }

  std::string message;
  std::vector<bievr::GnssTrajectoryOptimizer::StampedPose> lio;
  if (!readTum(options.lio_trajectory, lio, message)) {
    LOG(E, message);
    return 1;
  }
  std::vector<bievr::GnssTrajectoryOptimizer::PositionMeasurement> gnss;
  bievr::V3 origin_lla;
  if (!readGnss(options.bag, options.gnss_topic, gnss, origin_lla, message)) {
    LOG(E, message);
    return 1;
  }
  bievr::GnssTrajectoryOptimizer optimizer(config);
  bievr::GnssTrajectoryOptimizer::Stats stats;
  std::vector<bievr::GnssTrajectoryOptimizer::StampedPose> optimized;
  if (!optimizer.optimize(lio, gnss, optimized, stats, &message)) {
    LOG(E, message);
    return 1;
  }
  LOG(I, message);
  if (!writeOutputs(options, origin_lla, lio, optimized, stats, message)) {
    LOG(E, message);
    return 1;
  }
  LOG(I, message);
  return 0;
}
