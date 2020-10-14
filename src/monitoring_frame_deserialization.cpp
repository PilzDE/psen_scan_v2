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

#include "psen_scan_v2/monitoring_frame_deserialization.h"
#include "psen_scan_v2/diagnostics.h"

namespace psen_scan_v2
{
MonitoringFrameAdditionalFieldHeader::MonitoringFrameAdditionalFieldHeader(Id id, Length length)
  : id_(id), length_(length)
{
}

constexpr MonitoringFrameAdditionalFieldHeader::Id MonitoringFrameAdditionalFieldIds::SCAN_COUNTER;
constexpr MonitoringFrameAdditionalFieldHeader::Id MonitoringFrameAdditionalFieldIds::MEASURES;
constexpr MonitoringFrameAdditionalFieldHeader::Id MonitoringFrameAdditionalFieldIds::END_OF_FRAME;
constexpr MonitoringFrameAdditionalFieldHeader::Id MonitoringFrameAdditionalFieldIds::DIAGNOSTICS;

MonitoringFrameMsg deserialize_monitoring_frame(const MaxSizeRawData& data, const std::size_t& num_bytes)
{
  MonitoringFrameMsg msg;

  MaxSizeRawData tmp_data{ data };
  std::istringstream is(std::string(tmp_data.data(), tmp_data.size()));

  raw_processing::read(is, msg.device_status_);
  raw_processing::read(is, msg.op_code_);
  raw_processing::read(is, msg.working_mode_);
  raw_processing::read(is, msg.transaction_type_);
  raw_processing::read(is, msg.scanner_id_);

  raw_processing::read<uint16_t, TenthOfDegree>(is, msg.from_theta_);
  raw_processing::read<uint16_t, TenthOfDegree>(is, msg.resolution_);

  if (OP_CODE_MONITORING_FRAME != msg.op_code_)
  {
    // TODO: Get rid of the issue not to spam the system with this debug messages
    //       Would something like  ROS_DEBUG_THROTTLE(period, ...) be a good solution?
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Wrong Op Code!");
  }

  if (ONLINE_WORKING_MODE != msg.working_mode_)
  {
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Invalid working mode!");
  }

  if (GUI_MONITORING_TRANSACTION != msg.transaction_type_)
  {
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Invalid transaction type!");
  }

  if (MAX_SCANNER_ID < static_cast<uint8_t>(msg.scanner_id_))
  {
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Invalid Scanner id!");
  }

  bool end_of_frame{ false };
  while (!end_of_frame)
  {
    const MonitoringFrameAdditionalFieldHeader header{ readFieldHeader(is, num_bytes) };

    switch (header.id())
    {
      case MonitoringFrameAdditionalFieldIds::SCAN_COUNTER:
        if (header.length() != NUMBER_OF_BYTES_SCAN_COUNTER)
        {
          throw MonitoringFrameFormatErrorScanCounterUnexpectedSize(fmt::format(
              "Length of scan counter field is {}, but should be {}.", header.length(), NUMBER_OF_BYTES_SCAN_COUNTER));
        }
        raw_processing::read(is, msg.scan_counter_);
        break;

      case MonitoringFrameAdditionalFieldIds::MEASURES:
        raw_processing::readArray<uint16_t, double>(is,
                                                    msg.measures_,
                                                    header.length() / NUMBER_OF_BYTES_SINGLE_MEASURE,
                                                    [](uint16_t raw_element) { return raw_element / 1000.; });
        break;

      case MonitoringFrameAdditionalFieldIds::END_OF_FRAME:
        end_of_frame = true;
        break;

      case MonitoringFrameAdditionalFieldIds::DIAGNOSTICS:
        msg.diagnostic_messages_ = deserializeDiagnosticMessages(is);
        // WIP: TODO: Move elsewhere
        for (auto& elem : msg.diagnostic_messages_)
        {
          std::cerr << elem;
        }
        break;

      default:
        throw MonitoringFrameFormatError(
            fmt::format("Header Id {:#04x} unknown. Cannot read additional field of monitoring frame.", header.id()));
    }
  }
  return msg;
}

MonitoringFrameAdditionalFieldHeader readFieldHeader(std::istringstream& is, const std::size_t& max_num_bytes)
{
  MonitoringFrameAdditionalFieldId id;
  MonitoringFrameAdditionalFieldLength length;
  raw_processing::read(is, id);
  raw_processing::read(is, length);

  if (length >= max_num_bytes)
  {
    throw MonitoringFrameFormatError(
        fmt::format("Length given in header of additional field is too large: {}, id: {:#04x}", length, id));
  }
  if (length > 0)
  {
    length--;
  }
  return MonitoringFrameAdditionalFieldHeader(id, length);
}

std::vector<MonitoringFrameDiagnosticMessage> deserializeDiagnosticMessages(std::istringstream& is)
{
  std::vector<MonitoringFrameDiagnosticMessage> diagnostic_messages;

  std::array<uint8_t, DIAGNOSTIC_MESSAGE_RAW_UNUSED_DATA_OFFSET_IN_BYTES> reserved_diag_unused;
  raw_processing::read(is, reserved_diag_unused);

  for (auto& scanner_id : SCANNER_IDS)
  {
    for (size_t byte_n = 0; byte_n < DIAGNOSTIC_MESSAGE_RAW_LENGTH_FOR_ONE_DEVICE_IN_BYTES; byte_n++)
    {
      uint8_t raw_byte;
      raw_processing::read(is, raw_byte);
      std::bitset<8> raw_bits(raw_byte);

      for (size_t bit_n = 0; bit_n < raw_bits.size(); ++bit_n)
      {
        if (raw_bits.test(bit_n) && (DiagnosticCode::UNUSED != error_bits[byte_n][bit_n]))
        {
          diagnostic_messages.push_back(
              MonitoringFrameDiagnosticMessage(static_cast<ScannerId>(scanner_id), byte_n, bit_n));
        }
      }
    }
  }
  return diagnostic_messages;
}  // namespace psen_scan_v2
}  // namespace psen_scan_v2
