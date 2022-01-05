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

#include <cstdint>
#include <string>
#include <vector>

#include "psen_scan_v2_standalone/configuration/scanner_ids.h"
#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"

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

const io::PinData& Message::iOPinData() const
{
  if (io_pin_data_.is_initialized())
  {
    return io_pin_data_.get();
  }
  else
  {
    throw AdditionalFieldMissing("IO pin data");
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

bool Message::hasIOPinField() const
{
  return io_pin_data_.is_initialized();
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
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
