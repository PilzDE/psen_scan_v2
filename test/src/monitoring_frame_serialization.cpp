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

#include <ostream>

#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/monitoring_frame_serialization.h"
#include "psen_scan_v2/monitoring_frame_deserialization.h"

namespace psen_scan_v2
{
namespace monitoring_frame
{
DynamicSizeRawData serialize(const Message& frame)
{
  std::ostringstream os;

  raw_processing::write(os, DEFAULT_DEVICE_STATUS);
  raw_processing::write(os, OP_CODE_MONITORING_FRAME);
  raw_processing::write(os, ONLINE_WORKING_MODE);
  raw_processing::write(os, GUI_MONITORING_TRANSACTION);
  raw_processing::write(os, frame.scanner_id_);
  raw_processing::write(os, frame.from_theta_.value());
  raw_processing::write(os, frame.resolution_.value());

  additional_field_header_ids::AdditionalFieldHeader scan_counter_header(
      static_cast<additional_field_header_ids::AdditionalFieldHeader::Id>(
          additional_field_header_ids::HeaderID::SCAN_COUNTER),
      sizeof(frame.scan_counter_));
  writeFieldHeader(os, scan_counter_header);
  uint32_t scan_counter_header_payload = frame.scan_counter_;
  raw_processing::write(os, scan_counter_header_payload);

  if (frame.diagnostic_data_enabled_)
  {
    additional_field_header_ids::AdditionalFieldHeader diagnostic_data_field_header(
        static_cast<additional_field_header_ids::AdditionalFieldHeader::Id>(
            additional_field_header_ids::HeaderID::DIAGNOSTICS),
        diagnostics::raw_message::LENGTH_IN_BYTES);
    writeFieldHeader(os, diagnostic_data_field_header);
    diagnostics::raw_message::Type diagnostic_data_field_payload =
        diagnostics::serializeMessages(frame.diagnostic_messages_);
    raw_processing::write(os, diagnostic_data_field_payload);
  }

  additional_field_header_ids::AdditionalFieldHeader measures_header(
      static_cast<additional_field_header_ids::AdditionalFieldHeader::Id>(
          additional_field_header_ids::HeaderID::MEASURES),
      frame.measures_.size() * NUMBER_OF_BYTES_SINGLE_MEASURE);
  writeFieldHeader(os, measures_header);
  raw_processing::writeArray<uint16_t, double>(
      os, frame.measures_, [](double elem) { return (static_cast<uint16_t>(std::round(elem * 1000.))); });

  additional_field_header_ids::AdditionalFieldHeader::Id end_of_frame_header_id =
      static_cast<additional_field_header_ids::AdditionalFieldHeader::Id>(
          additional_field_header_ids::HeaderID::END_OF_FRAME);
  raw_processing::write(os, end_of_frame_header_id);

  uint8_t unknown_data_at_the_end_of_frame = 0;
  raw_processing::write(os, unknown_data_at_the_end_of_frame);
  raw_processing::write(os, unknown_data_at_the_end_of_frame);
  raw_processing::write(os, unknown_data_at_the_end_of_frame);

  return raw_processing::toArray<DynamicSizeRawData>(os);
}

constexpr size_t calculateIndexInRawDiagnosticData(const ScannerId& id, const diagnostics::ErrorLocation& location)
{
  return diagnostics::raw_message::UNUSED_OFFSET_IN_BYTES +
         (static_cast<uint8_t>(id) * diagnostics::raw_message::LENGTH_FOR_ONE_DEVICE_IN_BYTES) + location.getByte();
}

namespace diagnostics
{
raw_message::Type serializeMessages(const std::vector<Message>& messages)
{
  raw_message::Type raw_diagnostic_data{};

  for (const auto& elem : messages)
  {
    raw_diagnostic_data.at(calculateIndexInRawDiagnosticData(elem.getScannerId(), elem.getErrorLocation())) +=
        (1 << elem.getErrorLocation().getBit());
  }
  return raw_diagnostic_data;
}
}  // namespace diagnostics

void writeFieldHeader(std::ostringstream& os, const additional_field_header_ids::AdditionalFieldHeader& header)
{
  raw_processing::write(os, header.id());
  raw_processing::write<additional_field_header_ids::AdditionalFieldHeader::Length>(os, header.length() + 1);
}

}  // namespace monitoring_frame
}  // namespace psen_scan_v2
