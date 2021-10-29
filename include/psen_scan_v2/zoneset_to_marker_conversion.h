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

#include <vector>

#include <visualization_msgs/Marker.h>

#include "psen_scan_v2/ZoneSet.h"

namespace psen_scan_v2
{
std::vector<visualization_msgs::Marker> toMarkers(const ZoneSet& zoneset_config)
{
  // ToDo
  return { visualization_msgs::Marker() };
}

}  // namespace psen_scan_v2

#endif
