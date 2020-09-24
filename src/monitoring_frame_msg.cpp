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
#include "psen_scan_v2/monitoring_frame_format_error.h"
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

MonitoringFrameMsg MonitoringFrameMsg::fromRawData(const MaxSizeRawData& data)
{
  MonitoringFrameMsg msg;

  MaxSizeRawData tmp_data{ data };
  std::istringstream is(std::string(tmp_data.data(), tmp_data.size()));

  raw_processing::read(is, msg.device_status_fixed_);
  raw_processing::read(is, msg.op_code_fixed_);
  raw_processing::read(is, msg.working_mode_fixed_);
  raw_processing::read(is, msg.transaction_type_fixed_);
  raw_processing::read(is, msg.scanner_id_fixed_);
  readAngle(is, msg.from_theta_fixed_);
  readAngle(is, msg.resolution_fixed_);

  msg.checkFixedFields();

  bool end_of_frame{ false };
  while (!end_of_frame)
  {
    const FieldHeader header{ readFieldHeader(is) };

    switch (header.id())
    {
      case AdditionalFieldIds::SCAN_COUNTER:
        readScanCounter(is, msg.scan_counter_, header.length());
        break;

      case AdditionalFieldIds::MEASURES:
        readMeasures(is, msg.measures_, header.length());
        break;

      case AdditionalFieldIds::END_OF_FRAME:
        end_of_frame = true;
        break;

      default:
        std::ostringstream os;
        os << "Header Id " << std::hex << header.id() << " unknown. Cannot read additional field of monitoring frame.";
        throw MonitoringFrameFormatError(os.str());
    }
  }

  return msg;
}

void MonitoringFrameMsg::readAngle(std::istringstream& is, double& angle)
{
  uint16_t angle_in_tenth_degree;
  raw_processing::read(is, angle_in_tenth_degree);
  angle = tenthDegreeToRad(angle_in_tenth_degree);
}

FieldHeader MonitoringFrameMsg::readFieldHeader(std::istringstream& is)
{
  FieldId id;
  FieldLength length;
  raw_processing::read(is, id);
  raw_processing::read(is, length);
  length--;
  return FieldHeader(id, length);
}

void MonitoringFrameMsg::readScanCounter(std::istringstream& is, uint32_t& scan_counter, const FieldLength length)
{
  if (length != sizeof(scan_counter))
  {
    std::ostringstream os;
    os << "Length of scan counter field is " << length << ", but should be " << sizeof(scan_counter_) << ".";
    throw MonitoringFrameFormatError(os.str());
  }
  raw_processing::read(is, scan_counter);
}

void MonitoringFrameMsg::readMeasures(std::istringstream& is, std::vector<double>& measures, const FieldLength length)
{
  size_t bytes_per_sample = sizeof(uint16_t);
  size_t number_of_samples = length / bytes_per_sample;

  measures.resize(number_of_samples);

  for (unsigned i = 0; i < number_of_samples; i++)
  {
    uint16_t sample;
    raw_processing::read(is, sample);
    measures.at(i) = sample / 1000.;  // Convert to m
  }
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
