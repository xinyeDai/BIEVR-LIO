#ifndef BIEVR_LIO_ROS_SAVE_MAP_SERVICE_H_
#define BIEVR_LIO_ROS_SAVE_MAP_SERVICE_H_

#include <bievr_lio/pipeline.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include <memory>
#include <string>
#include <utility>

namespace bievr {

class SaveMapService {
 public:
  SaveMapService(ros::NodeHandle& private_nh, std::shared_ptr<const Pipeline> pipeline)
      : pipeline_(std::move(pipeline)) {
    server_ = private_nh.advertiseService("save_map_drifted", &SaveMapService::save, this);
  }

 private:
  bool save(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
    std::string written;
    size_t num_points = 0;
    response.success = pipeline_->saveMap("", &written, &num_points);
    response.message = response.success
                           ? "Saved " + std::to_string(num_points) + " points to " + written
                           : "Failed to save map; see the node log for the reason.";
    return true;
  }

  std::shared_ptr<const Pipeline> pipeline_;
  ros::ServiceServer server_;
};

}  // namespace bievr

#endif
