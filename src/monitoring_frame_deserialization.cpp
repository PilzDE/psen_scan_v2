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

#include <iostream>

#include "psen_scan_v2/monitoring_frame_deserialization.h"
#include "psen_scan_v2/diagnostic.h"

namespace psen_scan_v2
{
FieldHeader::FieldHeader(Id id, Length length) : id_(id), length_(length)
{
}

constexpr FieldHeader::Id AdditionalFieldIds::SCAN_COUNTER;
constexpr FieldHeader::Id AdditionalFieldIds::MEASURES;
constexpr FieldHeader::Id AdditionalFieldIds::END_OF_FRAME;
constexpr FieldHeader::Id AdditionalFieldIds::DIAGNOSTICS;

MonitoringFrameMsg deserialize_monitoring_frame(const MaxSizeRawData& data, const std::size_t& num_bytes)
{
  MonitoringFrameMsg msg;

  MaxSizeRawData tmp_data{ data };
  std::istringstream is(std::string(tmp_data.data(), tmp_data.size()));

  raw_processing::read(is, msg.device_status_fixed_);
  raw_processing::read(is, msg.op_code_fixed_);
  raw_processing::read(is, msg.working_mode_fixed_);
  raw_processing::read(is, msg.transaction_type_fixed_);
  raw_processing::read(is, msg.scanner_id_fixed_);

  raw_processing::read<uint16_t, TenthOfDegree>(is, msg.from_theta_fixed_);
  raw_processing::read<uint16_t, TenthOfDegree>(is, msg.resolution_fixed_);

  checkFixedFields(msg);

  bool end_of_frame{ false };
  while (!end_of_frame)
  {
    const FieldHeader header{ readFieldHeader(is, num_bytes) };

    switch (header.id())
    {
      case AdditionalFieldIds::SCAN_COUNTER:
        if (header.length() != NUMBER_OF_BYTES_SCAN_COUNTER)
        {
          throw MonitoringFrameFormatErrorScanCounterUnexpectedSize(fmt::format(
              "Length of scan counter field is {}, but should be {}.", header.length(), NUMBER_OF_BYTES_SCAN_COUNTER));
        }
        raw_processing::read(is, msg.scan_counter_);
        break;

      case AdditionalFieldIds::MEASURES:
        raw_processing::readArray<uint16_t, double>(is,
                                                    msg.measures_,
                                                    header.length() / NUMBER_OF_BYTES_SINGLE_MEASURE,
                                                    [](uint16_t raw_element) { return raw_element / 1000.; });
        break;

      case AdditionalFieldIds::END_OF_FRAME:
        end_of_frame = true;
        break;

      case AdditionalFieldIds::DIAGNOSTICS:
        std::array<char, 4> reserved_diag;
        raw_processing::read(is, reserved_diag);

        for (size_t m = 0; m < 4; ++m)
        {
          // std::cerr << "Device " << m << "\n";
          std::array<std::bitset<8>, 9> diagnostics_device_data;
          raw_processing::read<std::array<char, 9>, std::array<std::bitset<8>, 9> >(
              is, diagnostics_device_data, [](std::array<char, 9> raw) {
                std::array<std::bitset<8>, 9> diagnostics;
                for (size_t i = 0; i < 9; ++i)
                {
                  diagnostics[i] = std::bitset<8>(raw[i]);

                  // std::cerr << "[" << i << "]" << diagnostics[i].to_string() << "\n";
                }
                // std::cerr << "--------\n";

                return diagnostics;
              });

          msg.diagnostics_[m] = diagnostics_device_data;
          for (size_t byte_n = 0; byte_n < 9; ++byte_n)
          {
            for (size_t bit_n = 0; bit_n < msg.diagnostics_[m][byte_n].size(); ++bit_n)
            {
              if (msg.diagnostics_[m][byte_n].test(bit_n))
              {
                // Print nothing if Error bit on UNUSED field is set
                if (error_bits[byte_n][bit_n] != DiagnosticCode::UNUSED)
                {
                  // Internal and - bits are ambiguous, also print byte and bit
                  if (error_bits[byte_n][bit_n] == DiagnosticCode::_ ||
                      error_bits[byte_n][bit_n] == DiagnosticCode::INT)
                  {
                    std::cerr << error_code_to_string.at(error_bits[byte_n][bit_n]) << " byte: " << byte_n
                              << " bit_n:" << bit_n << "\n";
                  }
                  else
                  {
                    std::cerr << error_code_to_string.at(error_bits[byte_n][bit_n]);
                  }
                }
              }
            }
          }
        }
        break;

      default:
        throw MonitoringFrameFormatError(
            fmt::format("Header Id {:#04x} unknown. Cannot read additional field of monitoring frame.", header.id()));
    }
  }

  return msg;
}

FieldHeader readFieldHeader(std::istringstream& is, const std::size_t& max_num_bytes)
{
  FieldId id;
  FieldLength length;
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
  return FieldHeader(id, length);
}

void checkFixedFields(MonitoringFrameMsg& msg)
{
  if (OP_CODE_MONITORING_FRAME != msg.op_code_fixed_)
  {
    // TODO: Get rid of the issue not to spam the system with this debug messages
    //       Would something like  ROS_DEBUG_THROTTLE(period, ...) be a good solution?
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Wrong Op Code!");
  }

  if (ONLINE_WORKING_MODE != msg.working_mode_fixed_)
  {
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Invalid working mode!");
  }

  if (GUI_MONITORING_TRANSACTION != msg.transaction_type_fixed_)
  {
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Invalid transaction type!");
  }

  if (MAX_SCANNER_ID < msg.scanner_id_fixed_)
  {
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Invalid Scanner id!");
  }
}
}  // namespace psen_scan_v2
