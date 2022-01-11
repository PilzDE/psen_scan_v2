// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_ACTIVE_ZONESET_NODE_H
#define PSEN_SCAN_V2_ACTIVE_ZONESET_NODE_H

#include <string>
#include <vector>
#include <boost/optional.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

#include "psen_scan_v2/ZoneSetConfiguration.h"

namespace psen_scan_v2
{
static const std::string DEFAULT_ACTIVE_ZONESET_TOPIC = "active_zoneset";
static const std::string DEFAULT_ZONECONFIGURATION_TOPIC = "zoneconfiguration";
static const std::string DEFAULT_ZONESET_MARKER_TOPIC = "active_zoneset_marker";

/**
 * @brief ROS Node that continuously publishes a marker for the active_zoneset.
 *
 * subscribes to: ns/active_zoneset
 * subscribes to: ns/zoneconfiguration
 *
 * advertises: ns/active_zoneset_marker
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

public:
  void zonesetCallback(const ZoneSetConfiguration& zoneset_config);
  void activeZonesetCallback(const std_msgs::UInt8& zoneset_config);

private:
  void updateMarkers();
  bool isAllInformationAvailable() const;

  /*! deprecated: use ZoneSet activeZoneset() const instead */
  [[deprecated("use ZoneSet activeZoneset() const instead")]] ZoneSet getActiveZoneset() const;
  ZoneSet activeZoneset() const;

  void deleteLastMarkers();
  void addMarkers(std::vector<visualization_msgs::Marker>& new_markers);
  bool containLastMarkers(const std::vector<visualization_msgs::Marker>& new_markers);

private:
  ros::NodeHandle nh_;
  ros::Subscriber zoneset_subscriber_;
  ros::Subscriber active_zoneset_subscriber_;
  ros::Publisher zoneset_marker_;

  boost::optional<ZoneSetConfiguration> zoneset_config_;
  boost::optional<std_msgs::UInt8> active_zoneset_id_;
  std::vector<visualization_msgs::Marker> last_markers_;
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ACTIVE_ZONESET_NODE_H
