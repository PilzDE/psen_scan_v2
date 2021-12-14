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

#include <algorithm>
#include <stdexcept>
#include <vector>

#include <boost/optional.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

#include "psen_scan_v2/active_zoneset_node.h"
#include "psen_scan_v2/ZoneSetConfiguration.h"
#include "psen_scan_v2/zoneset_to_marker_conversion.h"

namespace psen_scan_v2
{
ActiveZonesetNode::ActiveZonesetNode(ros::NodeHandle& nh) : nh_(nh)
{
  zoneset_subscriber_ = nh_.subscribe(DEFAULT_ZONECONFIGURATION_TOPIC, 2, &ActiveZonesetNode::zonesetCallback, this);
  active_zoneset_subscriber_ =
      nh_.subscribe(DEFAULT_ACTIVE_ZONESET_TOPIC, 10, &ActiveZonesetNode::activeZonesetCallback, this);
  zoneset_marker_ = nh_.advertise<visualization_msgs::Marker>(DEFAULT_ZONESET_MARKER_TOPIC, 10);
}

void ActiveZonesetNode::zonesetCallback(const ZoneSetConfiguration& zoneset_config)
{
  zoneset_config_ = zoneset_config;
  updateMarkers();
}

void ActiveZonesetNode::activeZonesetCallback(const std_msgs::UInt8& active_zoneset_id)
{
  active_zoneset_id_ = active_zoneset_id;
  updateMarkers();
};

void ActiveZonesetNode::updateMarkers()
{
  if (isAllInformationAvailable())
  {
    try
    {
      auto new_markers = toMarkers(activeZoneset());
      if (!containLastMarkers(new_markers))
      {
        deleteLastMarkers();
      }
      addMarkers(new_markers);
    }
    catch (std::out_of_range const& e)
    {
      ROS_ERROR_STREAM_THROTTLE(1,
                                "Active zone " << static_cast<unsigned>(active_zoneset_id_->data)
                                               << " of your scanner does not exist in the provided configuration!");
      deleteLastMarkers();
    }
  }
}

bool ActiveZonesetNode::isAllInformationAvailable() const
{
  return active_zoneset_id_.is_initialized() && zoneset_config_.is_initialized();
}

ZoneSet ActiveZonesetNode::activeZoneset() const
{
  return zoneset_config_->zonesets.at(active_zoneset_id_->data);
}

void ActiveZonesetNode::addMarkers(std::vector<visualization_msgs::Marker>& new_markers)
{
  for (const auto& marker : new_markers)
  {
    zoneset_marker_.publish(marker);
  }
  last_markers_ = std::move(new_markers);
}

void ActiveZonesetNode::deleteLastMarkers()
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

bool ActiveZonesetNode::containLastMarkers(const std::vector<visualization_msgs::Marker>& new_markers)
{
  if (new_markers.size() < last_markers_.size())
  {
    return false;
  }
  return std::mismatch(last_markers_.begin(),
                       last_markers_.end(),
                       new_markers.begin(),
                       [](const auto& m1, const auto& m2) { return m1.ns == m2.ns; })
             .first == last_markers_.end();
}

}  // namespace psen_scan_v2
