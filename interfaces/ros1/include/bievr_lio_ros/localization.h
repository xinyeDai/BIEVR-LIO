#ifndef BIEVR_LIO_ROS_LOCALIZATION_H_
#define BIEVR_LIO_ROS_LOCALIZATION_H_

#include <bievr_lio/pipeline.h>
#include <bievr_localization/config_loader.h>
#include <bievr_localization/localizer.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "bievr_ros_common/conversions.h"

namespace bievr {

class Localization {
 public:
  Localization(ros::NodeHandle& nh, const std::shared_ptr<Pipeline>& pipeline,
               LocalizationConfig config, const std::string& topic_prefix)
      : nh_(nh), config_(std::move(config)), pipeline_(pipeline),
        localizer_(Localizer::create(config_.localizer)) {
    odom_frame_ = pipeline->config().map_frame;
    body_frame_ = pipeline->config().body_frame;
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(topic_prefix + "/loc/odom", 1, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>(topic_prefix + "/loc/path", 1, true);
    map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_prefix + "/loc/map", 1, true);
    status_pub_ = nh_.advertise<std_msgs::String>(topic_prefix + "/loc/status", 1, true);
  }

  bool start(std::string* message = nullptr) {
    if (!localizer_->start(message)) return false;
    initial_pose_sub_ = nh_.subscribe("/initialpose", 1, &Localization::onInitialPose, this);
    pose_timer_ = nh_.createTimer(period(config_.publish_frequency), &Localization::publishPose,
                                  this);
    map_timer_ = nh_.createTimer(ros::Duration(1.0), &Localization::publishMap, this);
    if (config_.publish_path) {
      path_timer_ = nh_.createTimer(period(config_.path_publish_frequency),
                                    &Localization::publishPath, this);
    }
    pipeline_->addFrameObserver(
        [this](uint64_t stamp, const Transform& pose, const Pointcloud& cloud) {
          {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            latest_odom_ = pose;
            latest_stamp_ = stamp;
            have_odom_ = true;
          }
          localizer_->addFrame(stamp, pose, cloud);
        });
    return true;
  }

  Localizer::Status status() const { return localizer_->status(); }
  Localizer::Stats stats() const { return localizer_->stats(); }
  bool canRelocalize() const { return localizer_->canRelocalize(); }

  // Saves the map-frame trajectory published on /loc/path in TUM format. The
  // output is placed beside the configured map bundle (normally Log/).
  bool saveTrajectory(std::string* message = nullptr) {
    namespace fs = std::filesystem;
    const fs::path map_path(config_.localizer.map_path);
    const fs::path output_dir =
        map_path.has_parent_path() ? map_path.parent_path() : fs::path("Log");
    const fs::path output_path = output_dir / "localization_tum.txt";

    Path snapshot;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      snapshot = trajectory_;
    }
    if (snapshot.poses.empty()) {
      if (message) *message = "No localized poses accumulated; localization trajectory not saved.";
      return false;
    }

    std::error_code ec;
    fs::create_directories(output_dir, ec);
    if (ec) {
      if (message) *message = "Failed to create localization log directory: " + ec.message();
      return false;
    }

    std::ofstream out(output_path);
    if (!out) {
      if (message) *message = "Failed to open '" + output_path.string() + "'.";
      return false;
    }
    out << std::fixed << std::setprecision(9);
    for (const auto& stamped : snapshot.poses) {
      const V3& p = stamped.pose.translation();
      const Quaternion q(stamped.pose.linear());
      out << nsToS(stamped.stamp) << " " << p.x() << " " << p.y() << " " << p.z() << " "
          << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    if (!out) {
      if (message) *message = "Failed while writing '" + output_path.string() + "'.";
      return false;
    }
    if (message) {
      *message = "Saved " + std::to_string(snapshot.poses.size()) +
                 " localized poses to '" + output_path.string() + "'.";
    }
    return true;
  }

 private:
  static ros::Duration period(double hz) { return ros::Duration(hz > 0.0 ? 1.0 / hz : 1.0); }

  void onInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != config_.localizer.map_frame) {
      LOG(W, "Ignoring /initialpose in frame '" << msg->header.frame_id << "'; expected '"
                                                  << config_.localizer.map_frame << "'.");
      return;
    }
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;
    localizer_->setInitialPose(Transform(Quaternion(q.w, q.x, q.y, q.z), Point(p.x, p.y, p.z)));
    LOG(I, "Accepted an initial pose at (" << p.x << ", " << p.y << ", " << p.z << ").");
  }

  void publishPose(const ros::TimerEvent&) {
    const auto status = localizer_->status();
    if (status != last_status_) {
      LOG(I, "Localization status: " << Localizer::toString(status));
      last_status_ = status;
    }
    std_msgs::String status_msg;
    status_msg.data = Localizer::toString(status);
    status_pub_.publish(status_msg);
    const auto correction = localizer_->correction();
    if (!correction.valid) return;

    Transform T_odom_body;
    uint64_t stamp = 0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!have_odom_) return;
      T_odom_body = latest_odom_;
      stamp = latest_stamp_;
      // The pose timer normally runs faster than LiDAR odometry. Publishing the
      // same sensor stamp repeatedly makes tf2 reject map->odom with
      // TF_REPEATED_DATA, so emit pose data once per new odometry frame.
      if (stamp == last_pose_publish_stamp_) return;
      last_pose_publish_stamp_ = stamp;
    }
    geometry_msgs::TransformStamped correction_msg;
    correction_msg.header.stamp.fromNSec(stamp);
    correction_msg.header.frame_id = config_.localizer.map_frame;
    correction_msg.child_frame_id = odom_frame_;
    transformToMsg(correction.T_map_odom, correction_msg.transform);
    tf_.sendTransform(correction_msg);

    const Transform T_map_body(Eigen::Isometry3d(correction.T_map_odom * T_odom_body));
    nav_msgs::Odometry odom;
    odom.header = correction_msg.header;
    odom.child_frame_id = body_frame_;
    transformToMsg(T_map_body, odom.pose.pose);
    odom_pub_.publish(odom);
    recordPath(stamp, T_map_body);
  }

  void recordPath(uint64_t stamp, const Transform& pose) {
    if (!config_.publish_path || stamp == last_path_stamp_) return;
    last_path_stamp_ = stamp;
    std::lock_guard<std::mutex> lock(path_mutex_);
    const StampedPose stamped{stamp, pose};
    trajectory_.poses.push_back(stamped);
    path_.poses.push_back(stamped);
    if (config_.path_max_poses > 0 && path_.poses.size() > config_.path_max_poses) {
      path_.poses.erase(path_.poses.begin(),
                        path_.poses.begin() + (path_.poses.size() - config_.path_max_poses));
    }
  }

  void publishPath(const ros::TimerEvent&) {
    Path snapshot;
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      if (path_.poses.empty()) return;
      snapshot = path_;
    }
    Header header;
    header.stamp = snapshot.poses.back().stamp;
    header.frame = config_.localizer.map_frame;
    nav_msgs::Path msg;
    pathToMsg(snapshot, header, msg);
    path_pub_.publish(msg);
  }

  void publishMap(const ros::TimerEvent&) {
    const uint64_t generation = localizer_->mapGeneration();
    if (generation == published_generation_) return;
    const Localizer::Cloud::ConstPtr map = localizer_->vizCloud();
    if (!map || map->empty()) return;
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = config_.localizer.map_frame;
    msg.header.stamp = ros::Time::now();
    map_pub_.publish(msg);
    published_generation_ = generation;
  }

  ros::NodeHandle nh_;
  LocalizationConfig config_;
  std::shared_ptr<Pipeline> pipeline_;
  std::unique_ptr<Localizer> localizer_;
  std::string odom_frame_, body_frame_;
  Localizer::Status last_status_ = Localizer::Status::NoMap;
  uint64_t published_generation_ = 0;
  std::mutex odom_mutex_, path_mutex_;
  Transform latest_odom_;
  uint64_t latest_stamp_ = 0, last_pose_publish_stamp_ = 0, last_path_stamp_ = 0;
  bool have_odom_ = false;
  // Complete map-frame localization result retained for the shutdown export.
  Path trajectory_;
  // Potentially bounded copy used for the live ROS path publication.
  Path path_;
  ros::Publisher odom_pub_, path_pub_, map_pub_, status_pub_;
  ros::Subscriber initial_pose_sub_;
  tf::TransformBroadcaster tf_;
  ros::Timer pose_timer_, map_timer_, path_timer_;
};

}  // namespace bievr

#endif
