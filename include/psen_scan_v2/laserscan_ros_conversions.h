// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PSEN_SCAN_V2_LASERSCAN_ROS_CONVERSIONS_H
#define PSEN_SCAN_V2_LASERSCAN_ROS_CONVERSIONS_H

#include <sensor_msgs/LaserScan.h>

#include "psen_scan_v2/scanner.h"

namespace psen_scan_v2
{
sensor_msgs::LaserScan toLaserScanMsg(const LaserScan& laserscan,
                                      const std::string& frame_id,
                                      const double x_axis_rotation,
                                      const ros::Time& timestamp = ros::Time::now())
{
  sensor_msgs::LaserScan ros_message;
  ros_message.header.stamp = timestamp;
  ros_message.header.frame_id = frame_id;
  ros_message.angle_min = -(laserscan.getMaxScanAngle().toRad() - x_axis_rotation);
  ros_message.angle_max = -(laserscan.getMinScanAngle().toRad() - x_axis_rotation);
  ros_message.angle_increment = laserscan.getScanResolution().toRad();

  // For now we set this to zero to hint that the ranges are actually ordered
  // from new to old. Thus applying some sort of interpolation using time_increment
  // could lead to unwanted results.
  ros_message.time_increment = 0;

  ros_message.scan_time = TIME_PER_SCAN_IN_S;
  ros_message.range_min = RANGE_MIN_IN_M;
  ros_message.range_max = RANGE_MAX_IN_M;

  // Note that the ranges field from the laserscan msg is field in reverse order
  // from the actual measurements. Thus the first element is the newest measurement and the last
  // element is the oldest
  ros_message.ranges.insert(
      ros_message.ranges.end(), laserscan.getMeasurements().crbegin(), laserscan.getMeasurements().crend());

  return ros_message;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_LASERSCAN_ROS_CONVERSIONS_H
