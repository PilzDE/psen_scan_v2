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
#include <memory>

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
  zoneset_subscriber_ = nh_.subscribe(DEFAULT_ZONESET_TOPIC, 4, &ActiveZonesetNode::zonesetCallback, this);
  active_zoneset_subscriber_ =
      nh_.subscribe(DEFAULT_ACTIVE_ZONESET_TOPIC, 4, &ActiveZonesetNode::activeZonesetCallback, this);
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
    try
    {
      auto new_markers = toMarkers(zoneset_config_->zonesets.at(active_zoneset_->data));
      sendNewMarkersAndDeleteOldOnes(std::move(new_markers));
    }
    catch (std::out_of_range const& e)
    {
      ROS_ERROR_STREAM_THROTTLE(1,
                                "Active zone " << active_zoneset_->data
                                               << " of your scanner does not exist in the provided configuration!");
      deleteMarkers();
    }
  }
}

void ActiveZonesetNode::sendNewMarkersAndDeleteOldOnes(std::vector<visualization_msgs::Marker> new_markers)
{
  if (!markersMatchLastMarkers(new_markers))
  {
    deleteMarkers();
  }
  for (const auto& marker : new_markers)
  {
    zoneset_marker_.publish(marker);
  }
  last_markers_ = std::move(new_markers);
}

void ActiveZonesetNode::deleteMarkers()
{
  for (const auto& lm : last_markers_)
  {
    auto marker = visualization_msgs::Marker();
    marker.action = visualization_msgs::Marker::DELETE;
    marker.ns = lm.ns;
    marker.id = 0;
    zoneset_marker_.publish(marker);
  }
  last_markers_.clear();
}

bool ActiveZonesetNode::markersMatchLastMarkers(const std::vector<visualization_msgs::Marker>& new_markers)
{
  if (last_markers_.size() == new_markers.size())
  {
    for (size_t i = 0; i < last_markers_.size(); i++)
    {
      if (last_markers_.at(i).ns != new_markers.at(i).ns)
      {
        return false;
      }
    }
    return true;
  }
  return false;
}

}  // namespace psen_scan_v2
