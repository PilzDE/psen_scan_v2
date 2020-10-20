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
#ifndef PSEN_SCAN_V2_MONITORING_FRAME_SERIALIZATION_H
#define PSEN_SCAN_V2_MONITORING_FRAME_SERIALIZATION_H

#include "psen_scan_v2/diagnostics.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/monitoring_frame_deserialization.h"
#include "psen_scan_v2/raw_scanner_data.h"

using namespace psen_scan_v2;

namespace psen_scan_v2
{
namespace monitoring_frame
{
DynamicSizeRawData serialize(const monitoring_frame::Message& frame);
namespace diagnostics
{
raw_message::Type serializeMessages(const std::vector<Message>& messages);
}  // namespace diagnostics
void writeFieldHeader(std::ostringstream& os, const monitoring_frame::additional_field::Header& header);
}  // namespace monitoring_frame
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_SERIALIZATION_H
