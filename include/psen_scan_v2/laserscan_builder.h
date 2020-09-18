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

#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/tenth_degree_conversion.h"

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
  void add(const MonitoringFrameMsg& frame);
  bool laserScanReady() const;
  LaserScan build();
  void reset();

private:
  std::vector<MonitoringFrameMsg> frames_;
  bool ready_{ false };
};

inline void LaserScanBuilder::add(const MonitoringFrameMsg& frame)
{
  if (!ready_)
  {
    frames_.front() = frame;
    ready_ = true;
  }
}

inline bool LaserScanBuilder::laserScanReady() const
{
  return ready_;
}

inline LaserScan LaserScanBuilder::build()
{
  const MonitoringFrameMsg frame = frames_.front();

  const double resolution = tenthDegreeToRad(frame.resolution());
  const double min_angle = tenthDegreeToRad(frame.fromTheta());
  const uint16_t number_of_samples = frame.measures().size();
  const double max_angle = min_angle + resolution * number_of_samples;

  LaserScan ret(resolution, min_angle, max_angle);

  ret.setMeasurements(frame.measures());

  reset();

  return ret;
}

inline LaserScanBuildFailure::LaserScanBuildFailure(const std::string& msg) : std::runtime_error(msg)
{
}

inline void LaserScanBuilder::reset()
{
  frames_.clear();
  ready_ = false;
}
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_LASERSCAN_BUILDER_H