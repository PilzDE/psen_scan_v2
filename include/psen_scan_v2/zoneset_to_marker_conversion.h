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

#ifndef PSEN_SCAN_V2_ZONESET_TO_MARKER_CONVERSION_H
#define PSEN_SCAN_V2_ZONESET_TO_MARKER_CONVERSION_H

#include <string>
#include <vector>

#include <fmt/format.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include "psen_scan_v2/ZoneSet.h"

#define TO_MARKER(zoneset_obj, polygon_type, polygon_index)                                                            \
  createMarker(fmt::format("active zoneset {}{} {}", #polygon_type, #polygon_index, getRangeInfo(zoneset_obj)),        \
               createRGBA(strcmp(#polygon_type, "muting") != 0,                                                        \
                          strcmp(#polygon_type, "warn") == 0,                                                          \
                          strcmp(#polygon_type, "muting") == 0,                                                        \
                          1),                                                                                          \
               zoneset_obj.header.frame_id,                                                                            \
               zoneset_obj.polygon_type##polygon_index.points,                                                         \
               0.01 * (strcmp(#polygon_type, "warn") == 0) + 0.02 * (strcmp(#polygon_type, "muting") == 0))

namespace psen_scan_v2
{
geometry_msgs::Point createPoint(const double& x, const double& y, const double& z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

std_msgs::ColorRGBA createRGBA(const float& r, const float& g, const float& b, const float& a)
{
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

visualization_msgs::Marker createMarker(const std::string& ns,
                                        const std_msgs::ColorRGBA& color,
                                        const std::string& frame_id,
                                        const std::vector<geometry_msgs::Point32>& points,
                                        const double& z_offset = 0)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = 0;
  marker.type = marker.TRIANGLE_LIST;
  marker.action = marker.ADD;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.4;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = z_offset;

  marker.points.reserve(3 * (points.size() - 1));
  marker.colors.reserve(points.size() - 1);

  for (std::size_t i = 1; i < points.size(); ++i)
  {
    marker.points.push_back(createPoint(0.0, 0.0, 0.0));
    marker.points.push_back(createPoint(points.at(i - 1).x, points.at(i - 1).y, 0.0));
    marker.points.push_back(createPoint(points.at(i).x, points.at(i).y, 0.0));
    marker.colors.push_back(color);
  }
  return marker;
}

std::string getRangeInfo(const ZoneSet& zoneset)
{
  std::string range_info = "";
  if (zoneset.speed_lower != 0 || zoneset.speed_upper != 0)
  {
    range_info = fmt::format("min:{:+} max:{:+}", zoneset.speed_lower, zoneset.speed_upper);
  }
  return range_info;
}

std::vector<visualization_msgs::Marker> toMarkers(const ZoneSet& zoneset)
{
  std::vector<visualization_msgs::Marker> return_vec;

  if (!zoneset.safety1.points.empty())
  {
    return_vec.push_back(TO_MARKER(zoneset, safety, 1));
  }
  if (!zoneset.safety2.points.empty())
  {
    return_vec.push_back(TO_MARKER(zoneset, safety, 2));
  }
  if (!zoneset.safety3.points.empty())
  {
    return_vec.push_back(TO_MARKER(zoneset, safety, 3));
  }
  if (!zoneset.warn1.points.empty())
  {
    return_vec.push_back(TO_MARKER(zoneset, warn, 1));
  }
  if (!zoneset.warn2.points.empty())
  {
    return_vec.push_back(TO_MARKER(zoneset, warn, 2));
  }
  if (!zoneset.muting1.points.empty())
  {
    return_vec.push_back(TO_MARKER(zoneset, muting, 1));
  }
  if (!zoneset.muting2.points.empty())
  {
    return_vec.push_back(TO_MARKER(zoneset, muting, 2));
  }

  return return_vec;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ZONESET_TO_MARKER_CONVERSION_H
