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

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/logging.h"
#include "psen_scan_v2/diagnostics.h"

namespace psen_scan_v2
{
TenthOfDegree MonitoringFrameMsg::fromTheta() const
{
  return from_theta_;
}

TenthOfDegree MonitoringFrameMsg::resolution() const
{
  return resolution_;
}

uint32_t MonitoringFrameMsg::scanCounter() const
{
  return scan_counter_;
}

std::vector<double> MonitoringFrameMsg::measures() const
{
  return measures_;
}

std::vector<MonitoringFrameDiagnosticMessage> MonitoringFrameMsg::diagnostic_messages() const
{
  return diagnostic_messages_;
}

bool MonitoringFrameMsg::operator==(const MonitoringFrameMsg& rhs) const
{
  return (fromTheta() == rhs.fromTheta() && resolution() == rhs.resolution() && scanCounter() == rhs.scanCounter() &&
          measures() == rhs.measures());
}

}  // namespace psen_scan_v2
