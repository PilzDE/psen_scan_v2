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

#include "psen_scan_v2_standalone/data_conversion_layer/raw_processing.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_deserialization.h"

#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_serialization.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace monitoring_frame
{
RawData serialize(const data_conversion_layer::monitoring_frame::Message& msg)
{
  std::ostringstream os;

  raw_processing::write(os, DEFAULT_DEVICE_STATUS);
  raw_processing::write(os, OP_CODE_MONITORING_FRAME);
  raw_processing::write(os, ONLINE_WORKING_MODE);
  raw_processing::write(os, GUI_MONITORING_TRANSACTION);
  raw_processing::write(os, msg.scanner_id_);
  raw_processing::write(os, msg.from_theta_.value());
  raw_processing::write(os, msg.resolution_.value());

  AdditionalFieldHeader scan_counter_header(
      static_cast<AdditionalFieldHeader::Id>(AdditionalFieldHeaderID::scan_counter), sizeof(msg.scan_counter_.get()));
  write(os, scan_counter_header);
  uint32_t scan_counter_header_payload = msg.scan_counter_.get();
  raw_processing::write(os, scan_counter_header_payload);

  if (msg.diagnostic_data_enabled_)
  {
    AdditionalFieldHeader diagnostic_data_field_header(
        static_cast<AdditionalFieldHeader::Id>(AdditionalFieldHeaderID::diagnostics),
        diagnostic::RAW_CHUNK_LENGTH_IN_BYTES);
    write(os, diagnostic_data_field_header);
    diagnostic::RawChunk diagnostic_data_field_payload = diagnostic::serialize(msg.diagnostic_messages_);
    data_conversion_layer::raw_processing::write(os, diagnostic_data_field_payload);
  }

  AdditionalFieldHeader measurements_header(
      static_cast<AdditionalFieldHeader::Id>(AdditionalFieldHeaderID::measurements),
      msg.measurements_.size() * NUMBER_OF_BYTES_SINGLE_MEASUREMENT);
  write(os, measurements_header);
  data_conversion_layer::raw_processing::writeArray<uint16_t, double>(os, msg.measurements_, [](double elem) {
    if (elem == std::numeric_limits<double>::infinity())
    {
      return NO_SIGNAL_ARRIVED;
    }
    return (static_cast<uint16_t>(std::round(elem * 1000.)));
  });

  if (!msg.intensities_.empty())
  {
    AdditionalFieldHeader intensities_header(
        static_cast<AdditionalFieldHeader::Id>(AdditionalFieldHeaderID::intensities),
        msg.intensities_.size() * NUMBER_OF_BYTES_SINGLE_INTENSITY);
    write(os, intensities_header);
    data_conversion_layer::raw_processing::writeArray<uint16_t, double>(
        os, msg.intensities_, [](double elem) { return (static_cast<uint16_t>(std::round(elem))); });
  }

  AdditionalFieldHeader::Id end_of_frame_header_id =
      static_cast<AdditionalFieldHeader::Id>(AdditionalFieldHeaderID::end_of_frame);
  data_conversion_layer::raw_processing::write(os, end_of_frame_header_id);

  uint8_t unknown_data_at_the_end_of_frame = 0;
  data_conversion_layer::raw_processing::write(os, unknown_data_at_the_end_of_frame);
  data_conversion_layer::raw_processing::write(os, unknown_data_at_the_end_of_frame);
  data_conversion_layer::raw_processing::write(os, unknown_data_at_the_end_of_frame);

  return data_conversion_layer::raw_processing::toArray<RawData>(os);
}

constexpr size_t calculateIndexInRawDiagnosticData(const configuration::ScannerId& id,
                                                   const diagnostic::ErrorLocation& location)
{
  return diagnostic::RAW_CHUNK_UNUSED_OFFSET_IN_BYTES +
         (static_cast<uint8_t>(id) * diagnostic::RAW_CHUNK_LENGTH_FOR_ONE_DEVICE_IN_BYTES) + location.getByte();
}

namespace diagnostic
{
RawChunk serialize(const std::vector<data_conversion_layer::monitoring_frame::diagnostic::Message>& messages)
{
  RawChunk raw_diagnostic_data{};

  for (const auto& elem : messages)
  {
    raw_diagnostic_data.at(calculateIndexInRawDiagnosticData(elem.getScannerId(), elem.getErrorLocation())) +=
        (1 << elem.getErrorLocation().getBit());
  }
  return raw_diagnostic_data;
}
}  // namespace diagnostic

void write(std::ostringstream& os, const AdditionalFieldHeader& header)
{
  data_conversion_layer::raw_processing::write(os, header.id());
  data_conversion_layer::raw_processing::write<AdditionalFieldHeader::Length>(os, header.length() + 1);
}

}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
