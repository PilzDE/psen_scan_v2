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

#include <bitset>
#include <iostream>

#include "psen_scan_v2/diagnostics.h"
#include "psen_scan_v2/monitoring_frame_deserialization.h"

namespace psen_scan_v2
{
namespace monitoring_frame
{
additional_field::Header::Header(Id id, Length length) : id_(id), length_(length)
{
}

FixedFields::FixedFields(DeviceStatus device_status,
                         OpCode op_code,
                         WorkingMode working_mode,
                         TransactionType transaction_type,
                         ScannerId scanner_id,
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

Message deserialize(const MaxSizeRawData& data, const std::size_t& num_bytes)
{
  Message msg;

  MaxSizeRawData tmp_data{ data };
  std::istringstream is(std::string(tmp_data.data(), tmp_data.size()));

  FixedFields frame_header = readFixedFields(is);

  msg.scanner_id_ = frame_header.scanner_id();
  msg.from_theta_ = frame_header.from_theta();
  msg.resolution_ = frame_header.resolution();

  bool end_of_frame{ false };
  while (!end_of_frame)
  {
    const additional_field::Header additional_header{ additional_field::read(is, num_bytes) };

    switch (static_cast<additional_field::HeaderID>(additional_header.id()))
    {
      case additional_field::HeaderID::SCAN_COUNTER:
        if (additional_header.length() != NUMBER_OF_BYTES_SCAN_COUNTER)
        {
          throw FormatErrorScanCounterUnexpectedSize(
              fmt::format("Length of scan counter field is {}, but should be {}.",
                          additional_header.length(),
                          NUMBER_OF_BYTES_SCAN_COUNTER));
        }
        raw_processing::read(is, msg.scan_counter_);
        break;

      case additional_field::HeaderID::MEASURES:
        raw_processing::readArray<uint16_t, double>(is,
                                                    msg.measures_,
                                                    additional_header.length() / NUMBER_OF_BYTES_SINGLE_MEASURE,
                                                    [](uint16_t raw_element) { return raw_element / 1000.; });
        break;

      case additional_field::HeaderID::END_OF_FRAME:
        end_of_frame = true;
        break;

      case additional_field::HeaderID::DIAGNOSTICS:
        msg.diagnostic_messages_ = deserializeDiagnosticMessages(is);
        msg.diagnostic_data_enabled_ = true;
        break;

      default:
        throw FormatError(fmt::format("Header Id {:#04x} unknown. Cannot read additional field of monitoring frame.",
                                      additional_header.id()));
    }
  }
  return msg;
}

additional_field::Header additional_field::read(std::istringstream& is, const std::size_t& max_num_bytes)
{
  additional_field::Header::Id id;
  additional_field::Header::Length length;
  raw_processing::read(is, id);
  raw_processing::read(is, length);

  if (length >= max_num_bytes)
  {
    throw FormatError(
        fmt::format("Length given in header of additional field is too large: {}, id: {:#04x}", length, id));
  }
  if (length > 0)
  {
    length--;
  }
  return additional_field::Header(id, length);
}

std::vector<diagnostics::Message> deserializeDiagnosticMessages(std::istringstream& is)
{
  std::vector<diagnostics::Message> diagnostic_messages;

  std::array<uint8_t, diagnostics::raw_message::UNUSED_OFFSET_IN_BYTES> reserved_diag_unused;
  raw_processing::read(is, reserved_diag_unused);

  for (auto& scanner_id : VALID_SCANNER_IDS)
  {
    for (size_t byte_n = 0; byte_n < diagnostics::raw_message::LENGTH_FOR_ONE_DEVICE_IN_BYTES; byte_n++)
    {
      uint8_t raw_byte;
      raw_processing::read(is, raw_byte);
      std::bitset<8> raw_bits(raw_byte);

      for (size_t bit_n = 0; bit_n < raw_bits.size(); ++bit_n)
      {
        if (raw_bits.test(bit_n) && (diagnostics::DiagnosticCode::UNUSED != diagnostics::error_bits[byte_n][bit_n]))
        {
          diagnostic_messages.push_back(
              diagnostics::Message(static_cast<ScannerId>(scanner_id), diagnostics::ErrorLocation(byte_n, bit_n)));
        }
      }
    }
  }
  return diagnostic_messages;
}

FixedFields readFixedFields(std::istringstream& is)
{
  FixedFields::DeviceStatus device_status;
  FixedFields::OpCode op_code;
  FixedFields::WorkingMode working_mode;
  FixedFields::TransactionType transaction_type;
  ScannerId scanner_id;
  FixedFields::FromTheta from_theta(0);
  FixedFields::Resolution resolution(0);

  raw_processing::read(is, device_status);
  raw_processing::read(is, op_code);
  raw_processing::read(is, working_mode);
  raw_processing::read(is, transaction_type);
  raw_processing::read(is, scanner_id);

  raw_processing::read<uint16_t, TenthOfDegree>(is, from_theta);
  raw_processing::read<uint16_t, TenthOfDegree>(is, resolution);

  if (OP_CODE_MONITORING_FRAME != op_code)
  {
    // TODO: Get rid of the issue not to spam the system with this debug messages
    //       Would something like  ROS_DEBUG_THROTTLE(period, ...) be a good solution?
    PSENSCAN_DEBUG("monitoring_frame::Message", "Wrong Op Code!");
  }

  if (ONLINE_WORKING_MODE != working_mode)
  {
    PSENSCAN_DEBUG("monitoring_frame::Message", "Invalid working mode!");
  }

  if (GUI_MONITORING_TRANSACTION != transaction_type)
  {
    PSENSCAN_DEBUG("monitoring_frame::Message", "Invalid transaction type!");
  }

  if (MAX_SCANNER_ID < static_cast<uint8_t>(scanner_id))
  {
    PSENSCAN_DEBUG("monitoring_frame::Message", "Invalid Scanner id!");
  }

  return FixedFields(device_status, op_code, working_mode, transaction_type, scanner_id, from_theta, resolution);
}
}  // namespace monitoring_frame
}  // namespace psen_scan_v2
