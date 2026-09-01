#ifndef BIEVR_LIO_ROS_LOOP_CLOSURE_H_
#define BIEVR_LIO_ROS_LOOP_CLOSURE_H_

#include <bievr_lio/pipeline.h>
#include <bievr_pgo/config_loader.h>
#include <bievr_pgo/loop_closer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <string>
#include <utility>

#include "bievr_ros_common/conversions.h"

namespace bievr {

class LoopClosure {
 public:
  LoopClosure(ros::NodeHandle& nh, ros::NodeHandle& private_nh,
              const std::shared_ptr<Pipeline>& pipeline, LoopClosureConfig config,
              const std::string& topic_prefix)
      : config_(std::move(config)), closer_(LoopCloser::Create(config_.closer)) {
    bundle_path_ = config_.bundle_path;
    child_frame_ = pipeline->config().body_frame;
    path_pub_ = nh.advertise<nav_msgs::Path>(topic_prefix + "/pgo/path", 1, true);
    odom_pub_ = nh.advertise<nav_msgs::Odometry>(topic_prefix + "/pgo/odom", 1, true);
    map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_prefix + "/pgo/map", 1, true);
    save_srv_ = private_nh.advertiseService("save_map_bundle", &LoopClosure::save, this);
    path_timer_ = nh.createTimer(period(config_.path_publish_frequency), &LoopClosure::publishPath,
                                 this);
    map_timer_ = nh.createTimer(period(config_.map_publish_frequency), &LoopClosure::publishMap,
                                this);
    pipeline->addFrameObserver(
        [this](uint64_t stamp, const Transform& pose, const Pointcloud& cloud) {
          closer_->addFrame(stamp, pose, cloud);
        });
  }

  LoopCloser::Stats stats() const { return closer_->stats(); }
  bool idle() const { return closer_->idle(); }
  bool saveBundle(std::string* message) const {
    return closer_->saveMapBundle(bundle_path_, message);
  }
  const std::string& bundlePath() const { return bundle_path_; }

 private:
  static ros::Duration period(double hz) { return ros::Duration(hz > 0.0 ? 1.0 / hz : 1.0); }

  bool save(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    response.success = closer_->saveMapBundle(bundle_path_, &response.message);
    return true;
  }

  void publishPath(const ros::TimerEvent&) {
    const auto keyframes = closer_->keyframes();
    if (keyframes.empty()) return;
    nav_msgs::Path path;
    path.header.frame_id = config_.closer.map_frame;
    path.header.stamp.fromNSec(keyframes.back().stamp);
    path.poses.reserve(keyframes.size());
    for (const auto& kf : keyframes) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = path.header.frame_id;
      pose.header.stamp.fromNSec(kf.stamp);
      transformToMsg(kf.pose, pose.pose);
      path.poses.push_back(pose);
    }
    path_pub_.publish(path);
    nav_msgs::Odometry odom;
    odom.header = path.header;
    odom.child_frame_id = child_frame_;
    transformToMsg(keyframes.back().pose, odom.pose.pose);
    odom_pub_.publish(odom);
  }

  void publishMap(const ros::TimerEvent&) {
    if (map_pub_.getNumSubscribers() == 0) return;
    const Pointcloud map = closer_->buildMap(config_.closer.map_save_resolution);
    if (map.empty()) return;
    sensor_msgs::PointCloud2 msg;
    pointCloudToMsg(map, msg);
    msg.header.frame_id = config_.closer.map_frame;
    msg.header.stamp = ros::Time::now();
    map_pub_.publish(msg);
  }

  LoopClosureConfig config_;
  std::unique_ptr<LoopCloser> closer_;
  std::string bundle_path_;
  std::string child_frame_;
  ros::Publisher path_pub_, odom_pub_, map_pub_;
  ros::ServiceServer save_srv_;
  ros::Timer path_timer_, map_timer_;
};

}  // namespace bievr

#endif
