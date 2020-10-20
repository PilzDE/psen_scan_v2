// Copyright (c) 2020 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_LASERSCAN_CONVERSIONS_H
#define PSEN_SCAN_V2_LASERSCAN_CONVERSIONS_H

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/monitoring_frame_msg.h"

namespace psen_scan_v2
{
static LaserScan toLaserScan(const monitoring_frame::MonitoringFrameMsg& frame)
{
  const auto resolution = frame.resolution();
  const auto min_angle = frame.fromTheta();
  const uint16_t number_of_samples = frame.measures().size();
  const auto max_angle = (frame.fromTheta() + frame.resolution() * (number_of_samples - 1u));

  LaserScan scan(resolution, min_angle, max_angle);
  scan.setMeasurements(frame.measures());

  return scan;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_LASERSCAN_CONVERSIONS_H
