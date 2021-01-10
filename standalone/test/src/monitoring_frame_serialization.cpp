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

#include <ostream>

#include "psen_scan_v2_standalone/raw_processing.h"
#include "psen_scan_v2_standalone/monitoring_frame_deserialization.h"

#include "psen_scan_v2_standalone/monitoring_frame_serialization.h"

namespace psen_scan_v2_standalone
{
namespace monitoring_frame
{
RawData serialize(const monitoring_frame::Message& frame)
{
  std::ostringstream os;

  raw_processing::write(os, DEFAULT_DEVICE_STATUS);
  raw_processing::write(os, OP_CODE_MONITORING_FRAME);
  raw_processing::write(os, ONLINE_WORKING_MODE);
  raw_processing::write(os, GUI_MONITORING_TRANSACTION);
  raw_processing::write(os, frame.scanner_id_);
  raw_processing::write(os, frame.from_theta_.value());
  raw_processing::write(os, frame.resolution_.value());

  additional_field::Header scan_counter_header(
      static_cast<additional_field::Header::Id>(additional_field::HeaderID::scan_counter), sizeof(frame.scan_counter_));
  write(os, scan_counter_header);
  uint32_t scan_counter_header_payload = frame.scan_counter_;
  raw_processing::write(os, scan_counter_header_payload);

  if (frame.diagnostic_data_enabled_)
  {
    additional_field::Header diagnostic_data_field_header(
        static_cast<additional_field::Header::Id>(additional_field::HeaderID::diagnostics),
        diagnostic::raw_message::LENGTH_IN_BYTES);
    write(os, diagnostic_data_field_header);
    diagnostic::raw_message::Field diagnostic_data_field_payload = diagnostic::serialize(frame.diagnostic_messages_);
    raw_processing::write(os, diagnostic_data_field_payload);
  }

  additional_field::Header measurements_header(
      static_cast<additional_field::Header::Id>(additional_field::HeaderID::measurements),
      frame.measurements_.size() * NUMBER_OF_BYTES_SINGLE_MEASUREMENT);
  write(os, measurements_header);
  raw_processing::writeArray<uint16_t, double>(
      os, frame.measurements_, [](double elem) { return (static_cast<uint16_t>(std::round(elem * 1000.))); });

  if (!frame.intensities_.empty())
  {
    additional_field::Header intensities_header(
        static_cast<additional_field::Header::Id>(additional_field::HeaderID::intensities),
        frame.intensities_.size() * NUMBER_OF_BYTES_SINGLE_INTENSITY);
    write(os, intensities_header);
    raw_processing::writeArray<uint16_t, double>(
        os, frame.intensities_, [](double elem) { return (static_cast<uint16_t>(std::round(elem))); });
  }

  additional_field::Header::Id end_of_frame_header_id =
      static_cast<additional_field::Header::Id>(additional_field::HeaderID::end_of_frame);
  raw_processing::write(os, end_of_frame_header_id);

  uint8_t unknown_data_at_the_end_of_frame = 0;
  raw_processing::write(os, unknown_data_at_the_end_of_frame);
  raw_processing::write(os, unknown_data_at_the_end_of_frame);
  raw_processing::write(os, unknown_data_at_the_end_of_frame);

  return raw_processing::toArray<RawData>(os);
}

constexpr size_t calculateIndexInRawDiagnosticData(const ScannerId& id, const diagnostic::ErrorLocation& location)
{
  return diagnostic::raw_message::UNUSED_OFFSET_IN_BYTES +
         (static_cast<uint8_t>(id) * diagnostic::raw_message::LENGTH_FOR_ONE_DEVICE_IN_BYTES) + location.getByte();
}

namespace diagnostic
{
raw_message::Field serialize(const std::vector<monitoring_frame::diagnostic::Message>& messages)
{
  raw_message::Field raw_diagnostic_data{};

  for (const auto& elem : messages)
  {
    raw_diagnostic_data.at(calculateIndexInRawDiagnosticData(elem.getScannerId(), elem.getErrorLocation())) +=
        (1 << elem.getErrorLocation().getBit());
  }
  return raw_diagnostic_data;
}
}  // namespace diagnostic

void write(std::ostringstream& os, const additional_field::Header& header)
{
  raw_processing::write(os, header.id());
  raw_processing::write<additional_field::Header::Length>(os, header.length() + 1);
}

}  // namespace monitoring_frame
}  // namespace psen_scan_v2_standalone
