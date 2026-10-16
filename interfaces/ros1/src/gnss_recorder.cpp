#include <bievr_lio_ros/gnss_io.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

class GnssRecorder {
 public:
  GnssRecorder() {
    ros::NodeHandle node;
    ros::NodeHandle params("~");
    std::string topic, directory;
    params.param<std::string>("gnss_recorder/gnss_topic", topic, "/gnss");
    params.param<std::string>("output_directory", directory, "Log/gnss");
    bool overwrite = false;
    params.param("overwrite", overwrite, false);
    const std::filesystem::path path = std::filesystem::absolute(directory);
    std::filesystem::create_directories(path);
    for (const auto& name : {"gnss_raw.txt", "gnss_enu.txt", "gnss_enu_tum.txt",
                             "gnss_pose_raw.txt", "gnss_pose_enu.txt", "gnss_pose_enu_tum.txt"}) {
      if (!overwrite && std::filesystem::exists(path / name)) {
        throw std::runtime_error("Output already exists: " + (path / name).string() +
                                 "; choose another directory or set ~overwrite:=true.");
      }
    }
    raw_.open(path / "gnss_raw.txt");
    enu_.open(path / "gnss_enu.txt");
    tum_.open(path / "gnss_enu_tum.txt");
    pose_raw_.open(path / "gnss_pose_raw.txt");
    pose_enu_.open(path / "gnss_pose_enu.txt");
    pose_tum_.open(path / "gnss_pose_enu_tum.txt");
    if (!raw_ || !enu_ || !tum_ || !pose_raw_ || !pose_enu_ || !pose_tum_) throw std::runtime_error("Cannot open GNSS output files.");
    raw_ << "# timestamp latitude_deg longitude_deg altitude_m status service covariance_type"
            " cov00 cov01 cov02 cov10 cov11 cov12 cov20 cov21 cov22\n";
    enu_ << "# timestamp east_m north_m up_m variance_e_m2 variance_n_m2 variance_u_m2\n";
    tum_ << "# timestamp east_m north_m up_m qx qy qz qw (identity: GNSS has no attitude)\n";
    pose_raw_ << "# timestamp latitude_deg longitude_deg altitude_m qx qy qz qw (original message quaternion)\n";
    pose_enu_ << "# timestamp east_m north_m up_m variance_e_m2 variance_n_m2 variance_u_m2\n"
              << "# PoseStamped has no covariance; variance columns use defaults 1 1 4 m^2.\n";
    pose_tum_ << "# timestamp east_m north_m up_m qx qy qz qw (identity: position reference only)\n";
    for (auto* output : {&raw_, &enu_, &tum_, &pose_raw_, &pose_enu_, &pose_tum_})
      *output << std::fixed << std::setprecision(12);
    std::vector<double> origin;
    if (params.hasParam("origin_lla")) {
      if (!params.getParam("origin_lla", origin) || origin.size() != 3 ||
          !std::isfinite(origin[0]) || std::abs(origin[0]) > 90.0 ||
          !std::isfinite(origin[1]) || std::abs(origin[1]) > 180.0 ||
          !std::isfinite(origin[2])) {
        throw std::runtime_error("~origin_lla must be [latitude_deg, longitude_deg, altitude_m].");
      }
      setOrigin(origin[0], origin[1], origin[2]);
    }
    raw_.flush();
    enu_.flush();
    tum_.flush();
    pose_raw_.flush();
    pose_enu_.flush();
    pose_tum_.flush();
    pose_subscriber_ = node.subscribe("/gnss_pose", 10000, &GnssRecorder::recordPose, this);
    subscriber_ = node.subscribe(topic, 10000, &GnssRecorder::record, this);
    ROS_INFO_STREAM("Recording NavSatFix from " << topic << " and LLA PoseStamped from /gnss_pose to " << path);
  }

  bool failed() const { return failed_; }

 private:
  void setOrigin(double latitude, double longitude, double altitude) {
    converter_ = std::make_unique<GeographicLib::LocalCartesian>(latitude, longitude, altitude);
    for (auto* output : {&enu_, &tum_, &pose_enu_, &pose_tum_}) {
      *output << "# origin_lla " << latitude << ' ' << longitude << ' ' << altitude << '\n';
      output->flush();
    }
    ROS_INFO_STREAM("GNSS ENU origin (lat lon alt): " << std::setprecision(12)
                    << latitude << ' ' << longitude << ' ' << altitude);
  }

  void record(const sensor_msgs::NavSatFix::ConstPtr& fix) {
    bievr_ros::writeStamp(raw_, fix->header.stamp);
    raw_ << ' ' << fix->latitude << ' ' << fix->longitude << ' ' << fix->altitude
         << ' ' << static_cast<int>(fix->status.status) << ' ' << fix->status.service
         << ' ' << static_cast<int>(fix->position_covariance_type);
    for (double value : fix->position_covariance) raw_ << ' ' << value;
    raw_ << '\n';
    raw_.flush();
    if (bievr_ros::validFix(*fix)) {
      if (!converter_) setOrigin(fix->latitude, fix->longitude, fix->altitude);
      double east, north, up;
      converter_->Forward(fix->latitude, fix->longitude, fix->altitude, east, north, up);
      const auto variance = bievr_ros::gnssVariance(*fix);
      bievr_ros::writeStamp(enu_, fix->header.stamp);
      enu_ << ' ' << east << ' ' << north << ' ' << up << ' ' << variance[0]
           << ' ' << variance[1] << ' ' << variance[2] << '\n';
      bievr_ros::writeStamp(tum_, fix->header.stamp);
      tum_ << ' ' << east << ' ' << north << ' ' << up << " 0 0 0 1\n";
      enu_.flush();
      tum_.flush();
    } else {
      ROS_WARN_THROTTLE(5.0, "Invalid NavSatFix saved to raw file only.");
    }
    checkOutputs();
  }

  void recordPose(const geometry_msgs::PoseStamped::ConstPtr& pose) {
    const auto& p = pose->pose.position;
    const auto& q = pose->pose.orientation;
    bievr_ros::writeStamp(pose_raw_, pose->header.stamp);
    pose_raw_ << ' ' << p.x << ' ' << p.y << ' ' << p.z << ' ' << q.x << ' '
              << q.y << ' ' << q.z << ' ' << q.w << '\n';
    pose_raw_.flush();
    // This receiver encodes latitude/longitude/ellipsoid height in position.x/y/z.
    // PoseStamped frame_id does not change these field semantics.
    if (std::isfinite(p.x) && std::abs(p.x) <= 90.0 &&
        std::isfinite(p.y) && std::abs(p.y) <= 180.0 && std::isfinite(p.z)) {
      if (!converter_) setOrigin(p.x, p.y, p.z);
      double east, north, up;
      converter_->Forward(p.x, p.y, p.z, east, north, up);
      bievr_ros::writeStamp(pose_enu_, pose->header.stamp);
      pose_enu_ << ' ' << east << ' ' << north << ' ' << up << " 1 1 4\n";
      bievr_ros::writeStamp(pose_tum_, pose->header.stamp);
      // Only position is used as truth. The receiver's attitude reference is unspecified.
      pose_tum_ << ' ' << east << ' ' << north << ' ' << up << " 0 0 0 1\n";
      pose_enu_.flush();
      pose_tum_.flush();
    } else {
      ROS_WARN_THROTTLE(5.0, "Invalid /gnss_pose LLA saved to raw file only.");
    }
    checkOutputs();
  }

  void checkOutputs() {
    if (!raw_ || !enu_ || !tum_ || !pose_raw_ || !pose_enu_ || !pose_tum_) {
      ROS_FATAL("Failed writing GNSS output; stopping recorder.");
      ros::shutdown();
      failed_ = true;
    }
  }

  ros::Subscriber subscriber_, pose_subscriber_;
  std::ofstream raw_, enu_, tum_, pose_raw_, pose_enu_, pose_tum_;
  std::unique_ptr<GeographicLib::LocalCartesian> converter_;
  bool failed_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "bievr_gnss_recorder");
  try {
    ros::NodeHandle params("~");
    bool enabled = true;
    params.param("gnss_recorder/enable", enabled, true);
    if (!enabled) {
      ROS_INFO("GNSS recording disabled by config.");
      return 0;
    }
    GnssRecorder recorder;
    ros::spin();
    return recorder.failed() ? 1 : 0;
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM(error.what());
    return 1;
  }
}
