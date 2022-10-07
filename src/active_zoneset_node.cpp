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
#include <string>
#include <vector>

#include <boost/optional.hpp>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
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
  zoneset_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(DEFAULT_ZONESET_MARKER_ARRAY_TOPIC, 10);
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
      addMarkers(new_markers);
    }
    catch (std::out_of_range const& e)
    {
      ROS_ERROR_STREAM_THROTTLE(1,
                                "Active zone " << static_cast<unsigned>(active_zoneset_id_->data)
                                               << " of your scanner does not exist in the provided configuration!");
    }
    addDeleteMessageForUnusedLastMarkers();
    publishCurrentMarkers();
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

// LCOV_EXCL_START
ZoneSet ActiveZonesetNode::getActiveZoneset() const
{
  return this->activeZoneset();
}
// LCOV_EXCL_STOP

void ActiveZonesetNode::addMarkers(std::vector<visualization_msgs::Marker>& new_markers)
{
  for (const auto& marker : new_markers)
  {
    current_markers_.push_back(marker);
  }
}

void ActiveZonesetNode::addDeleteMessageForUnusedLastMarkers()
{
  for (const auto& lm : last_markers_)
  {
    auto has_lm_namespace = [&lm](visualization_msgs::Marker& x) { return x.ns == lm.ns; };
    if (std::find_if(current_markers_.begin(), current_markers_.end(), has_lm_namespace) == current_markers_.end())
    {  // not in current markers -> delete it
      auto marker = visualization_msgs::Marker();
      marker.action = visualization_msgs::Marker::DELETE;
      marker.ns = lm.ns;
      marker.id = lm.id;
      current_markers_.push_back(marker);
    }
  }
  last_markers_.clear();
}

void ActiveZonesetNode::publishCurrentMarkers()
{
  auto ma = visualization_msgs::MarkerArray();
  ma.markers = current_markers_;
  zoneset_markers_.publish(ma);
  last_markers_ = std::move(current_markers_);
}

}  // namespace psen_scan_v2
