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

#include "psen_scan_v2_standalone/configuration/scanner_ids.h"
#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
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
configuration::ScannerId Message::scannerId() const
{
  return scanner_id_;
}

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
    throw AdditionalFieldMissing("Scan counter");
  }
}

uint8_t Message::activeZoneset() const
{
  if (active_zoneset_.is_initialized())
  {
    return active_zoneset_.get();
  }
  else
  {
    throw AdditionalFieldMissing("Active zoneset");
  }
}

const std::vector<double>& Message::measurements() const
{
  if (measurements_.is_initialized())
  {
    return measurements_.get();
  }
  else
  {
    throw AdditionalFieldMissing("Measurements");
  }
}

const std::vector<double>& Message::intensities() const
{
  if (intensities_.is_initialized())
  {
    return intensities_.get();
  }
  else
  {
    throw AdditionalFieldMissing("Intensities");
  }
}

std::vector<diagnostic::Message> Message::diagnosticMessages() const
{
  if (diagnostic_messages_.is_initialized())
  {
    return diagnostic_messages_.get();
  }
  else
  {
    throw AdditionalFieldMissing("Diagnostic messages");
  }
}

bool Message::hasScanCounterField() const
{
  return scan_counter_.is_initialized();
}

bool Message::hasActiveZonesetField() const
{
  return active_zoneset_.is_initialized();
}

bool Message::hasMeasurementsField() const
{
  return measurements_.is_initialized();
}

bool Message::hasIntensitiesField() const
{
  return intensities_.is_initialized();
}

bool Message::hasDiagnosticMessagesField() const
{
  return diagnostic_messages_.is_initialized();
}

#define FORMAT_IF_INITIALIZED(arg) arg.is_initialized() ? fmt::format("{}", arg.get()) : "_"
#define FORMAT_RANGE_IF_INITIALIZED(arg) arg.is_initialized() ? util::formatRange(arg.get()) : "_"

std::ostream& operator<<(std::ostream& os, const Message& msg)
{
  os << fmt::format("monitoring_frame::Message(fromTheta = {} deg, resolution = {} deg, scanCounter = "
                    "{}, active_zoneset = {}, measurements = {}, intensities = {}, diagnostics = {})",
                    msg.from_theta_.value() / 10.,
                    msg.resolution_.value() / 10.,
                    FORMAT_IF_INITIALIZED(msg.scan_counter_),
                    FORMAT_IF_INITIALIZED(msg.active_zoneset_),
                    FORMAT_RANGE_IF_INITIALIZED(msg.measurements_),
                    FORMAT_RANGE_IF_INITIALIZED(msg.intensities_),
                    FORMAT_RANGE_IF_INITIALIZED(msg.diagnostic_messages_));
  return os;
}
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
