#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/NavSatFix.h>

#include <array>
#include <cmath>
#include <iomanip>
#include <ostream>

namespace bievr_ros {

inline bool validFix(const sensor_msgs::NavSatFix& fix) {
  return fix.status.status >= sensor_msgs::NavSatStatus::STATUS_FIX &&
         std::isfinite(fix.latitude) && std::abs(fix.latitude) <= 90.0 &&
         std::isfinite(fix.longitude) && std::abs(fix.longitude) <= 180.0 &&
         std::isfinite(fix.altitude);
}

inline std::array<double, 3> gnssVariance(const sensor_msgs::NavSatFix& fix) {
  std::array<double, 3> variance{1.0, 1.0, 4.0};
  if (fix.position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
    variance = {fix.position_covariance[0], fix.position_covariance[4],
                fix.position_covariance[8]};
    for (double value : variance) {
      if (!std::isfinite(value) || value < 0.0) return {1.0, 1.0, 4.0};
    }
  }
  return variance;
}

// Preserve the original ROS timestamp without rounding through double seconds.
inline void writeStamp(std::ostream& out, const ros::Time& stamp) {
  out << stamp.sec << '.' << std::setfill('0') << std::setw(9) << stamp.nsec
      << std::setfill(' ');
}

}  // namespace bievr_ros
