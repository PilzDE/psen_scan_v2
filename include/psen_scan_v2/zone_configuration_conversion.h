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

#ifndef PSEN_SCAN_V2_ZONE_CONFIGURATION_CONVERSION_H
#define PSEN_SCAN_V2_ZONE_CONFIGURATION_CONVERSION_H

#include "psen_scan_v2/ZoneSet.h"
#include "psen_scan_v2/ZoneSetConfiguration.h"
#include "psen_scan_v2_standalone/configuration/zoneset_configuration.h"

using ZoneSetStandalone = psen_scan_v2_standalone::configuration::ZoneSet;
using ZoneSetConfigurationStandalone = psen_scan_v2_standalone::configuration::ZoneSetConfiguration;

static inline double deg_to_rad(double deg)
{
  return deg * M_PI / 180.0;
}

geometry_msgs::Polygon fromPolar(const std::vector<unsigned long>& radii_vec_mm, const double phi_inc)
{
  geometry_msgs::Polygon polygon;
  double phi = 0;
  for (const auto& r : radii_vec_mm)
  {
    geometry_msgs::Point32 point;

    point.x = (r / 1000.) * std::cos(phi);
    point.y = (r / 1000.) * std::sin(phi);
    point.z = 0;

    polygon.points.push_back(point);

    phi += phi_inc;
  }

  return polygon;
}

psen_scan_v2::ZoneSet toMsg(const ZoneSetStandalone& zoneset)
{
  psen_scan_v2::ZoneSet zoneset_msg;

  zoneset_msg.safety1 = fromPolar(zoneset.safety1_, deg_to_rad(0.5));
  zoneset_msg.safety2 = fromPolar(zoneset.safety2_, deg_to_rad(0.5));
  zoneset_msg.safety3 = fromPolar(zoneset.safety3_, deg_to_rad(0.5));
  zoneset_msg.warn1 = fromPolar(zoneset.warn1_, deg_to_rad(0.5));
  zoneset_msg.warn2 = fromPolar(zoneset.warn2_, deg_to_rad(0.5));
  zoneset_msg.muting1 = fromPolar(zoneset.muting1_, deg_to_rad(0.5));
  zoneset_msg.muting2 = fromPolar(zoneset.muting2_, deg_to_rad(0.5));

  if (zoneset.speed_range_)
  {
    zoneset_msg.speed_lower = zoneset.speed_range_->min_;
    zoneset_msg.speed_upper = zoneset.speed_range_->max_;
  }

  return zoneset_msg;
}

psen_scan_v2::ZoneSetConfiguration toMsg(const ZoneSetConfigurationStandalone& zoneset_configuration)
{
  psen_scan_v2::ZoneSetConfiguration zoneset_config_msg;
  for (const auto& z : zoneset_configuration.zonesets_)
  {
    zoneset_config_msg.zonesets.push_back(toMsg(z));
  }
  return zoneset_config_msg;
}

#endif  // PSEN_SCAN_V2_ZONE_CONFIGURATION_CONVERSION_H