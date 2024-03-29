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
#ifndef PSEN_SCAN_V2_MONITORING_FRAME_SERIALIZATION_H
#define PSEN_SCAN_V2_MONITORING_FRAME_SERIALIZATION_H

#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_constants.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_deserialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace monitoring_frame
{
RawData serialize(const data_conversion_layer::monitoring_frame::Message& msg);
namespace diagnostic
{
RawChunk serialize(const std::vector<data_conversion_layer::monitoring_frame::diagnostic::Message>& messages);
}  // namespace diagnostic
namespace io
{
RawChunk serialize(const PinData& pin_data);
}  // namespace io
void write(std::ostringstream& os, const data_conversion_layer::monitoring_frame::AdditionalFieldHeader& header);
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_SERIALIZATION_H
