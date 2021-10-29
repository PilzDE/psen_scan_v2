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

#ifndef PSEN_SCAN_V2_ACTIVE_ZONESET_H
#define PSEN_SCAN_V2_ACTIVE_ZONESET_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace psen_scan_v2
{
static const std::string DEFAULT_ZONESET_MARKER_TOPIC = "active_zoneset_marker";

/**
 * @brief ROS Node that continuously publishes a marker for the active_zoneset.
 *
 * subscribes to: ns/active_zoneset
 * subscribes to: ns/zonsesetconfiguration
 *
 * advertizes: ns/active_zoneset_marker
 */
class ActiveZonesetNode
{
public:
  /**
   * @brief Constructor.
   *
   * @param nh Node handle for the ROS node on which the scanner topic is advertised.
   */
  ActiveZonesetNode(ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;
  ros::Publisher zoneset_marker_;
};

ActiveZonesetNode::ActiveZonesetNode(ros::NodeHandle& nh) : nh_(nh)
{
  zoneset_marker_ = nh_.advertise<visualization_msgs::Marker>(DEFAULT_ZONESET_MARKER_TOPIC, 1);
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ACTIVE_ZONESET_H
