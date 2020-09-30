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

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/logging.h"

namespace psen_scan_v2
{
constexpr FieldHeader::Id AdditionalFieldIds::SCAN_COUNTER;
constexpr FieldHeader::Id AdditionalFieldIds::MEASURES;
constexpr FieldHeader::Id AdditionalFieldIds::END_OF_FRAME;

FieldHeader::FieldHeader(Id id, Length length) : id_(id), length_(length)
{
}

MonitoringFrameMsg MonitoringFrameMsg::fromRawData(const MaxSizeRawData& data, const std::size_t& num_bytes)
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

  msg.checkFixedFields();

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

      default:
        throw MonitoringFrameFormatError(
            fmt::format("Header Id {:#04x} unknown. Cannot read additional field of monitoring frame.", header.id()));
    }
  }

  return msg;
}

FieldHeader MonitoringFrameMsg::readFieldHeader(std::istringstream& is, const std::size_t& max_num_bytes)
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

void MonitoringFrameMsg::checkFixedFields()
{
  if (OP_CODE_MONITORING_FRAME != op_code_fixed_)
  {
    // TODO: Get rid of the issue not to spam the system with this debug messages
    //       Would something like  ROS_DEBUG_THROTTLE(period, ...) be a good solution?
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Wrong Op Code!");
  }

  if (ONLINE_WORKING_MODE != working_mode_fixed_)
  {
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Invalid working mode!");
  }

  if (GUI_MONITORING_TRANSACTION != transaction_type_fixed_)
  {
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Invalid transaction type!");
  }

  if (MAX_SCANNER_ID < scanner_id_fixed_)
  {
    PSENSCAN_DEBUG("MonitoringFrameMsg", "Invalid Scanner id!");
  }
}

}  // namespace psen_scan_v2
