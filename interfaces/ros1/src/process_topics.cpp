#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/synchronizer.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>
#include <topic_tools/shape_shifter.h>

#include "bievr_lio/config_loader.h"
#include "bievr_lio_ros/localization.h"
#include "bievr_lio_ros/loop_closure.h"
#include "bievr_lio_ros/publisher.h"
#include "bievr_lio_ros/save_map_service.h"
#include "bievr_ros_common/conversions.h"
#ifdef BIEVR_WITH_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#ifdef BIEVR_WITH_LIVOX2
#include <livox_ros_driver2/CustomMsg.h>
#endif

int main(int argc, char** argv) {
  srand(1);
  // ros::init strips ROS-specific arguments (remappings, __name:=, etc.) from
  // argv in place, leaving our config-file flags for loadConfigFromArgs.
  ros::init(argc, argv, "bievr_lio_topic_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bievr::Config config;
  if (!bievr::loadConfigFromArgs({argv, argv + argc}, config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  std::shared_ptr<bievr::Pipeline> pipeline =
      std::make_shared<bievr::Pipeline>(config.pipeline_config);
  bievr::Synchronizer synchronizer(pipeline);
  bievr::Publisher lio_pub(nh, pipeline, "bievr_lio");
  bievr::SaveMapService save_map_service(private_nh, pipeline);

  bievr::LoopClosureConfig loop_config;
  if (!bievr::loadLoopClosureConfig(config.yaml_paths, loop_config)) return -1;
  private_nh.param("loop_closure_enable", loop_config.enable, loop_config.enable);
  private_nh.param("bundle_path", loop_config.bundle_path, loop_config.bundle_path);
  std::unique_ptr<bievr::LoopClosure> loop_closure;
  if (loop_config.enable) {
    loop_closure.reset(new bievr::LoopClosure(nh, private_nh, pipeline, loop_config, "bievr_lio"));
  }

  bievr::LocalizationConfig localization_config;
  if (!bievr::loadLocalizationConfig(config.yaml_paths, localization_config)) return -1;
  private_nh.param("localization_enable", localization_config.enable, localization_config.enable);
  private_nh.param("map_path", localization_config.localizer.map_path,
                   localization_config.localizer.map_path);
  std::unique_ptr<bievr::Localization> localization;
  if (localization_config.enable) {
    if (loop_config.enable) {
      LOG(E, "loop_closure and localization cannot be enabled at the same time.");
      return -1;
    }
    localization.reset(
        new bievr::Localization(nh, pipeline, localization_config, "bievr_lio"));
    std::string message;
    if (!localization->start(&message)) {
      LOG(E, "Failed to start localization: " << message);
      return -1;
    }
  }

  ros::Subscriber pointcloud_sub = nh.subscribe<topic_tools::ShapeShifter>(
      config.topic_config.pointcloud_topic, 100000,
      [&](const topic_tools::ShapeShifter::ConstPtr& msg) {
        // Detect the message type by name
        std::string msg_type = msg->getDataType();
        LOG_FIRST(I, 1, "First message type received: " << msg_type);

        if (msg_type == "sensor_msgs/PointCloud2") {
          // Deserialize into sensor_msgs::PointCloud2
          sensor_msgs::PointCloud2::ConstPtr pc_msg = msg->instantiate<sensor_msgs::PointCloud2>();
          if (pc_msg) {
            bievr::StampedIntensityPointcloud pointcloud;
            bievr::msgToPointcloud(*pc_msg, pointcloud);
            synchronizer.addPointcloud(pointcloud);
          } else {
            LOG(E, "Failed to convert ShapeShifter to PointCloud2");
          }
        }
#ifdef BIEVR_WITH_LIVOX
        else if (msg_type == "livox_ros_driver/CustomMsg") {
          // Deserialize into livox_ros_driver::CustomMsg
          livox_ros_driver::CustomMsg::ConstPtr livox_msg =
              msg->instantiate<livox_ros_driver::CustomMsg>();
          if (livox_msg) {
            bievr::StampedIntensityPointcloud pointcloud;
            bievr::msgToPointcloud(*livox_msg, pointcloud);
            synchronizer.addPointcloud(pointcloud);
          } else {
            LOG(E, "Failed to convert ShapeShifter to livox_ros_driver CustomMsg");
          }
        }
#endif
#ifdef BIEVR_WITH_LIVOX2
        else if (msg_type == "livox_ros_driver2/CustomMsg") {
          // Deserialize into livox_ros_driver2::CustomMsg
          livox_ros_driver2::CustomMsg::ConstPtr livox2_msg =
              msg->instantiate<livox_ros_driver2::CustomMsg>();
          if (livox2_msg) {
            bievr::StampedIntensityPointcloud pointcloud;
            bievr::msgToPointcloud(*livox2_msg, pointcloud);
            synchronizer.addPointcloud(pointcloud);
          } else {
            LOG(E, "Failed to convert ShapeShifter to livox_ros_driver2 CustomMsg");
          }
        }
#endif
        else {
          LOG(W, "Received unsupported message type");
        }
      });

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(
      config.topic_config.imu_topic, 100000, [&](const sensor_msgs::Imu::ConstPtr& msg) {
        bievr::ImuMeasurement imu;
        bievr::msgToImuMeasurement(*msg, imu);
        synchronizer.addImu(imu);
      });

  // Keep LiDAR and IMU callbacks serialized. Loop detection, optimization and
  // localization ICP have their own worker threads inside the core modules.
  ros::spin();

  return 0;
}
