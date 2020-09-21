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
#ifndef PSEN_SCAN_V2_LASERSCAN_BUILDER_H
#define PSEN_SCAN_V2_LASERSCAN_BUILDER_H

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/monitoring_frame_msg.h"

namespace psen_scan_v2
{
class LaserScanBuildFailure : public std::runtime_error
{
public:
  LaserScanBuildFailure(const std::string& msg = "Error while building laser scan");
};

class LaserScanBuilder
{
public:
  static LaserScan build(const MonitoringFrameMsg& frame);
};

inline LaserScan LaserScanBuilder::build(const MonitoringFrameMsg& frame)
{
  const double resolution = tenthDegreeToRad(frame.resolution());
  const double min_angle = tenthDegreeToRad(frame.fromTheta());
  const uint16_t number_of_samples = frame.measures().size();
  const double max_angle = min_angle + resolution * number_of_samples;

  LaserScan scan(resolution, min_angle, max_angle);
  scan.setMeasurements(frame.measures());

  return scan;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_LASERSCAN_BUILDER_H
