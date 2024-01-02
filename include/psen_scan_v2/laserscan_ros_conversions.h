// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/laserscan.h"

namespace psen_scan_v2
{
using namespace psen_scan_v2_standalone;

sensor_msgs::LaserScan toLaserScanMsg(const LaserScan& laserscan,
                                      const std::string& frame_id,
                                      const double x_axis_rotation)
{
  sensor_msgs::LaserScan ros_message;
  if (laserscan.timestamp() < 0)
  {
    throw std::invalid_argument("Laserscan message has an invalid timestamp: " + std::to_string(laserscan.timestamp()));
  }
  ros_message.header.stamp = ros::Time{}.fromNSec(laserscan.timestamp());
  ros_message.header.frame_id = frame_id;
  ros_message.angle_min = laserscan.minScanAngle().toRad() - x_axis_rotation;
  ros_message.angle_max = laserscan.maxScanAngle().toRad() - x_axis_rotation;
  ros_message.angle_increment = laserscan.scanResolution().toRad();

  ros_message.time_increment = configuration::TIME_PER_SCAN_IN_S / (2 * M_PI) * laserscan.scanResolution().toRad();

  ros_message.scan_time = configuration::TIME_PER_SCAN_IN_S;
  ros_message.range_min = configuration::RANGE_MIN_IN_M;
  ros_message.range_max = configuration::RANGE_MAX_IN_M;

  ros_message.ranges.insert(
      ros_message.ranges.begin(), laserscan.measurements().cbegin(), laserscan.measurements().cend());

  ros_message.intensities.insert(
      ros_message.intensities.begin(), laserscan.intensities().cbegin(), laserscan.intensities().cend());

  return ros_message;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_LASERSCAN_ROS_CONVERSIONS_H
