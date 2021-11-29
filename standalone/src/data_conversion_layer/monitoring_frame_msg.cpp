// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg_builder.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/util/logging.h"
#include "psen_scan_v2_standalone/util/format_range.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace monitoring_frame
{
util::TenthOfDegree Message::fromTheta() const
{
  return from_theta_;
}

util::TenthOfDegree Message::resolution() const
{
  return resolution_;
}

uint32_t Message::scanCounter() const
{
  if (scan_counter_.is_initialized())
  {
    return scan_counter_.get();
  }
  else
  {
    throw ScanCounterMissing();
  }
}

uint8_t Message::activeZoneset() const
{
  return active_zoneset_;
}

const std::vector<double>& Message::measurements() const
{
  return measurements_;
}

const std::vector<double>& Message::intensities() const
{
  return intensities_;
}

std::vector<diagnostic::Message> Message::diagnosticMessages() const
{
  return diagnostic_messages_;
}

bool Message::operator==(const Message& rhs) const
{
  return (fromTheta() == rhs.fromTheta() && resolution() == rhs.resolution() && scanCounter() == rhs.scanCounter() &&
          activeZoneset() == rhs.activeZoneset() && measurements() == rhs.measurements() &&
          intensities() == rhs.intensities() && diagnosticMessages() == rhs.diagnosticMessages());
}

MessageBuilder Message::create()
{
  return MessageBuilder();
};

std::ostream& operator<<(std::ostream& os, const Message& msg)
{
  os << fmt::format("monitoring_frame::Message(fromTheta = {} deg, resolution = {} deg, scanCounter = "
                    "{}, active_zoneset = {}, measurements = {}, intensities = {}, diagnostics = {})",
                    msg.fromTheta().value() / 10.,
                    msg.resolution().value() / 10.,
                    msg.scanCounter(),
                    msg.activeZoneset(),
                    util::formatRange(msg.measurements()),
                    util::formatRange(msg.intensities()),
                    util::formatRange(msg.diagnosticMessages()));
  return os;
}
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
