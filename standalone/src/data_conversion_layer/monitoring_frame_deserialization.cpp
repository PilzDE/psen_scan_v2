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

#include <array>
#include <bitset>
#include <functional>
#include <istream>
#include <sstream>
#include <vector>

#include <fmt/format.h>

#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_deserialization.h"

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
namespace monitoring_frame
{
using namespace std::placeholders;

AdditionalFieldHeader::AdditionalFieldHeader(Id id, Length length) : id_(id), length_(length)
{
}

FixedFields::FixedFields(DeviceStatus device_status,
                         OpCode op_code,
                         WorkingMode working_mode,
                         TransactionType transaction_type,
                         configuration::ScannerId scanner_id,
                         FromTheta from_theta,
                         Resolution resolution)
  : device_status_(device_status)
  , op_code_(op_code)
  , working_mode_(working_mode)
  , transaction_type_(transaction_type)
  , scanner_id_(scanner_id)
  , from_theta_(from_theta)
  , resolution_(resolution)
{
}

static constexpr double toMeter(const uint16_t& value)
{
  if ((value == NO_SIGNAL_ARRIVED) || (value == SIGNAL_TOO_LATE))
  {
    return std::numeric_limits<double>::infinity();
  }
  return static_cast<double>(value) / 1000.;
}

static constexpr double toIntensities(const uint16_t& value)
{
  // Neglegt the first two bytes.
  uint16_t retval{ value };
  return static_cast<double>(retval & 0b0011111111111111);
}

monitoring_frame::Message deserialize(const data_conversion_layer::RawData& data, const std::size_t& num_bytes)
{
  data_conversion_layer::monitoring_frame::Message msg;

  std::stringstream ss;
  ss.write(data.data(), num_bytes);

  FixedFields frame_header = readFixedFields(ss);

  msg.scanner_id_ = frame_header.scanner_id();
  msg.from_theta_ = frame_header.from_theta();
  msg.resolution_ = frame_header.resolution();

  bool end_of_frame{ false };
  while (!end_of_frame)
  {
    const AdditionalFieldHeader additional_header{ readAdditionalField(ss, num_bytes) };

    switch (static_cast<AdditionalFieldHeaderID>(additional_header.id()))
    {
      case AdditionalFieldHeaderID::scan_counter:
        if (additional_header.length() != NUMBER_OF_BYTES_SCAN_COUNTER)
        {
          throw ScanCounterUnexpectedSize(fmt::format("Length of scan counter field is {}, but should be {}.",
                                                      additional_header.length(),
                                                      NUMBER_OF_BYTES_SCAN_COUNTER));
        }
        uint32_t scan_counter_read_buffer;
        raw_processing::read<uint32_t>(ss, scan_counter_read_buffer);
        msg.scan_counter_ = scan_counter_read_buffer;
        break;

      case AdditionalFieldHeaderID::measurements: {
        const size_t num_measurements{ static_cast<size_t>(additional_header.length()) /
                                       NUMBER_OF_BYTES_SINGLE_MEASUREMENT };
        raw_processing::readArray<uint16_t, double>(ss, msg.measurements_, num_measurements, toMeter);
        break;
      }
      case AdditionalFieldHeaderID::end_of_frame:
        end_of_frame = true;
        break;

      case AdditionalFieldHeaderID::diagnostics:
        msg.diagnostic_messages_ = diagnostic::deserializeMessages(ss);
        msg.diagnostic_data_enabled_ = true;
        break;

      case AdditionalFieldHeaderID::intensities: {
        const size_t num_measurements{ static_cast<size_t>(additional_header.length()) /
                                       NUMBER_OF_BYTES_SINGLE_MEASUREMENT };
        raw_processing::readArray<uint16_t, double>(
            ss, msg.intensities_, num_measurements, std::bind(toIntensities, _1));
        break;
      }
      default:
        throw DecodingFailure(fmt::format(
            "Header Id {:#04x} unknown. Cannot read additional field of monitoring frame.", additional_header.id()));
    }
  }
  return msg;
}

AdditionalFieldHeader readAdditionalField(std::istream& is, const std::size_t& max_num_bytes)
{
  auto const id = raw_processing::read<AdditionalFieldHeader::Id>(is);
  auto length = raw_processing::read<AdditionalFieldHeader::Length>(is);

  if (length >= max_num_bytes)
  {
    throw DecodingFailure(
        fmt::format("Length given in header of additional field is too large: {}, id: {:#04x}", length, id));
  }
  if (length > 0)
  {
    length--;
  }
  return AdditionalFieldHeader(id, length);
}

namespace diagnostic
{
std::vector<diagnostic::Message> deserializeMessages(std::istream& is)
{
  std::vector<diagnostic::Message> diagnostic_messages;

  // Read-in unused data fields
  raw_processing::read<std::array<uint8_t, diagnostic::RAW_CHUNK_UNUSED_OFFSET_IN_BYTES>>(is);

  for (const auto& scanner_id : configuration::VALID_SCANNER_IDS)
  {
    for (size_t byte_n = 0; byte_n < diagnostic::RAW_CHUNK_LENGTH_FOR_ONE_DEVICE_IN_BYTES; byte_n++)
    {
      const auto raw_byte = raw_processing::read<uint8_t>(is);
      const std::bitset<8> raw_bits(raw_byte);

      for (size_t bit_n = 0; bit_n < raw_bits.size(); ++bit_n)
      {
        if (raw_bits.test(bit_n) && (diagnostic::ErrorType::unused != diagnostic::error_bits[byte_n][bit_n]))
        {
          diagnostic_messages.push_back(diagnostic::Message(static_cast<configuration::ScannerId>(scanner_id),
                                                            diagnostic::ErrorLocation(byte_n, bit_n)));
        }
      }
    }
  }
  return diagnostic_messages;
}
}  // namespace diagnostic

FixedFields readFixedFields(std::istream& is)
{
  const auto device_status = raw_processing::read<FixedFields::DeviceStatus>(is);
  const auto op_code = raw_processing::read<FixedFields::OpCode>(is);
  const auto working_mode = raw_processing::read<FixedFields::WorkingMode>(is);
  const auto transaction_type = raw_processing::read<FixedFields::TransactionType>(is);
  const auto scanner_id = raw_processing::read<configuration::ScannerId>(is);

  const auto from_theta = raw_processing::read<int16_t, FixedFields::FromTheta>(is);
  const auto resolution = raw_processing::read<int16_t, FixedFields::Resolution>(is);

  // LCOV_EXCL_START
  if (OP_CODE_MONITORING_FRAME != op_code)
  {
    PSENSCAN_ERROR_THROTTLE(
        0.1, "monitoring_frame::Message", "Unexpected opcode during deserialization of MonitoringFrame.");
  }

  if (ONLINE_WORKING_MODE != working_mode)
  {
    PSENSCAN_ERROR_THROTTLE(0.1, "monitoring_frame::Message", "Invalid working mode (not online)");
  }

  if (GUI_MONITORING_TRANSACTION != transaction_type)
  {
    PSENSCAN_ERROR_THROTTLE(0.1, "monitoring_frame::Message", "Invalid transaction type.");
  }

  if (MAX_SCANNER_ID < static_cast<uint8_t>(scanner_id))
  {
    PSENSCAN_ERROR_THROTTLE(0.1, "monitoring_frame::Message", "Invalid Scanner id.");
  }
  // LCOV_EXCL_STOP

  return FixedFields(device_status, op_code, working_mode, transaction_type, scanner_id, from_theta, resolution);
}
}  // namespace monitoring_frame
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone
