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

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2/diagnostics.h"
#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/logging.h"
#include "psen_scan_v2/format_range.h"

namespace psen_scan_v2
{
TenthOfDegree monitoring_frame::Message::fromTheta() const
{
  return from_theta_;
}

TenthOfDegree monitoring_frame::Message::resolution() const
{
  return resolution_;
}

uint32_t monitoring_frame::Message::scanCounter() const
{
  if (scan_counter_.is_initialized())
  {
    return scan_counter_.get();
  }
  else
  {
    throw monitoring_frame::ScanCounterMissing();
  }
}

const std::vector<double>& monitoring_frame::Message::measurements() const
{
  return measurements_;
}

const std::vector<double>& monitoring_frame::Message::intensities() const
{
  return intensities_;
}

std::vector<monitoring_frame::diagnostic::Message> monitoring_frame::Message::diagnosticMessages() const
{
  return diagnostic_messages_;
}

bool monitoring_frame::Message::operator==(const monitoring_frame::Message& rhs) const
{
  return (fromTheta() == rhs.fromTheta() && resolution() == rhs.resolution() && scanCounter() == rhs.scanCounter() &&
          measurements() == rhs.measurements() && intensities() == rhs.intensities() &&
          diagnosticMessages() == rhs.diagnosticMessages());
}

namespace monitoring_frame
{
std::ostream& operator<<(std::ostream& os, const psen_scan_v2::monitoring_frame::Message& msg)
{
  os << fmt::format("monitoring_frame::Message(fromTheta = {} deg, resolution = {} deg, scanCounter = "
                    "{}, measurements = {}, intensities = {}, diagnostics = {})",
                    msg.fromTheta().value() / 10.,
                    msg.resolution().value() / 10.,
                    msg.scanCounter(),
                    formatRange(msg.measurements()),
                    formatRange(msg.intensities()),
                    formatRange(msg.diagnosticMessages()));
  return os;
}
}  // namespace monitoring_frame
}  // namespace psen_scan_v2
