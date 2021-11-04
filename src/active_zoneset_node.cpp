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

#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

#include "psen_scan_v2/active_zoneset_node.h"
#include "psen_scan_v2/config_server_node.h"
#include "psen_scan_v2/ZoneSetConfiguration.h"
#include "psen_scan_v2/zoneset_to_marker_conversion.h"

namespace psen_scan_v2
{
ActiveZonesetNode::ActiveZonesetNode(ros::NodeHandle& nh) : nh_(nh)
{
  zoneset_subscriber_ = nh_.subscribe(DEFAULT_ZONESET_TOPIC, 2, &ActiveZonesetNode::zonesetCallback, this);
  active_zoneset_subscriber_ = nh_.subscribe("active_zoneset", 2, &ActiveZonesetNode::activeZonesetCallback, this);
  zoneset_marker_ = nh_.advertise<visualization_msgs::Marker>(DEFAULT_ZONESET_MARKER_TOPIC, 10);
}

void ActiveZonesetNode::zonesetCallback(const ZoneSetConfiguration& zoneset_config)
{
  zoneset_config_ = zoneset_config;
  sendMarkersWhenAllInformationIsAvailable();
}

void ActiveZonesetNode::activeZonesetCallback(const std_msgs::UInt8& active_zoneset)
{
  active_zoneset_ = active_zoneset;
  sendMarkersWhenAllInformationIsAvailable();
};

void ActiveZonesetNode::sendMarkersWhenAllInformationIsAvailable()
{
  if (active_zoneset_.is_initialized() && zoneset_config_.is_initialized())
  {
    const auto markers = toMarkers(zoneset_config_->zonesets.at(active_zoneset_->data));
    for (const auto& marker : markers)
    {
      zoneset_marker_.publish(marker);
    }
  }
}

}  // namespace psen_scan_v2
