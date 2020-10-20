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
DynamicSizeRawData serialize(const MonitoringFrameMsg& frame)
{
  std::ostringstream os;

  raw_processing::write(os, DEFAULT_DEVICE_STATUS);
  raw_processing::write(os, OP_CODE_MONITORING_FRAME);
  raw_processing::write(os, ONLINE_WORKING_MODE);
  raw_processing::write(os, GUI_MONITORING_TRANSACTION);
  raw_processing::write(os, frame.scanner_id_);
  raw_processing::write(os, frame.from_theta_.value());
  raw_processing::write(os, frame.resolution_.value());

  MonitoringFrameAdditionalFieldHeader scan_counter_header(
      static_cast<MonitoringFrameAdditionalFieldHeader::Id>(
          monitoring_frame_additional_field_header_ids::HeaderID::SCAN_COUNTER),
      sizeof(frame.scan_counter_));
  writeFieldHeader(os, scan_counter_header);
  uint32_t scan_counter_header_payload = frame.scan_counter_;
  raw_processing::write(os, scan_counter_header_payload);

  if (frame.diagnostic_data_enabled_)
  {
    MonitoringFrameAdditionalFieldHeader diagnostic_data_field_header(
        static_cast<MonitoringFrameAdditionalFieldHeader::Id>(
            monitoring_frame_additional_field_header_ids::HeaderID::DIAGNOSTICS),
        RAW_DIAGNOSTIC_MESSAGE_LENGTH_IN_BYTES);
    writeFieldHeader(os, diagnostic_data_field_header);
    RawDiagnosticMsg diagnostic_data_field_payload = serializeDiagnosticMessages(frame.diagnostic_messages_);
    raw_processing::write(os, diagnostic_data_field_payload);
  }

  MonitoringFrameAdditionalFieldHeader measures_header(
      static_cast<MonitoringFrameAdditionalFieldHeader::Id>(
          monitoring_frame_additional_field_header_ids::HeaderID::MEASURES),
      frame.measures_.size() * NUMBER_OF_BYTES_SINGLE_MEASURE);
  writeFieldHeader(os, measures_header);
  raw_processing::writeArray<uint16_t, double>(
      os, frame.measures_, [](double elem) { return (static_cast<uint16_t>(std::round(elem * 1000.))); });

  MonitoringFrameAdditionalFieldHeader::Id end_of_frame_header_id =
      static_cast<MonitoringFrameAdditionalFieldHeader::Id>(
          monitoring_frame_additional_field_header_ids::HeaderID::END_OF_FRAME);
  raw_processing::write(os, end_of_frame_header_id);

  uint8_t unknown_data_at_the_end_of_frame = 0;
  raw_processing::write(os, unknown_data_at_the_end_of_frame);
  raw_processing::write(os, unknown_data_at_the_end_of_frame);
  raw_processing::write(os, unknown_data_at_the_end_of_frame);

  return raw_processing::toArray<DynamicSizeRawData>(os);
}

constexpr size_t calculateIndexInRawDiagnosticData(const ScannerId& id, const ErrorLocation& location)
{
  return RAW_DIAGNOSTIC_MESSAGE_UNUSED_OFFSET_IN_BYTES +
         (static_cast<uint8_t>(id) * RAW_DIAGNOSTIC_MESSAGE_LENGTH_FOR_ONE_DEVICE_IN_BYTES) + location.getByte();
}

RawDiagnosticMsg serializeDiagnosticMessages(const std::vector<MonitoringFrameDiagnosticMessage>& messages)
{
  RawDiagnosticMsg raw_diagnostic_data{};

  for (const auto& elem : messages)
  {
    raw_diagnostic_data.at(calculateIndexInRawDiagnosticData(elem.getScannerId(), elem.getErrorLocation())) +=
        (1 << elem.getErrorLocation().getBit());
  }
  return raw_diagnostic_data;
}

void writeFieldHeader(std::ostringstream& os, const MonitoringFrameAdditionalFieldHeader& header)
{
  raw_processing::write(os, header.id());
  raw_processing::write<MonitoringFrameAdditionalFieldHeader::Length>(os, header.length() + 1);
}

}  // namespace psen_scan_v2
