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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_MONITORING_FRAME_MSG_HELPER_H
#define PSEN_SCAN_V2_STANDALONE_TEST_MONITORING_FRAME_MSG_HELPER_H

#include <ostream>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/util/format_range.h"

#define FORMAT_PROPERTY_IF_CONDITION(msg, prop, cond) msg.cond() ? fmt::format("{}", msg.prop()) : "_"
#define FORMAT_RANGE_PROPERTY_IF_CONDITION(msg, prop, cond) msg.cond() ? util::formatRange(msg.prop()) : "_"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace monitoring_frame
{
static std::ostream& operator<<(std::ostream& os, const Message& msg)
{
  os << fmt::format("monitoring_frame::Message(fromTheta = {} deg, resolution = {} deg, scanCounter = "
                    "{}, active_zoneset = {}, measurements = {}, intensities = {}, diagnostics = {})",
                    msg.fromTheta().value() / 10.,
                    msg.resolution().value() / 10.,
                    FORMAT_PROPERTY_IF_CONDITION(msg, scanCounter, hasScanCounterField),
                    FORMAT_PROPERTY_IF_CONDITION(msg, activeZoneset, hasActiveZonesetField),
                    FORMAT_RANGE_PROPERTY_IF_CONDITION(msg, measurements, hasMeasurementsField),
                    FORMAT_RANGE_PROPERTY_IF_CONDITION(msg, intensities, hasIntensitiesField),
                    FORMAT_RANGE_PROPERTY_IF_CONDITION(msg, diagnosticMessages, hasDiagnosticMessagesField));
  return os;
}
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_MONITORING_FRAME_MSG_HELPER_H
