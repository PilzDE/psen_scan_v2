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
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "psen_scan_v2/monitoring_frame_format_error.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/raw_scanner_data.h"

namespace psen_scan_v2
{
constexpr FieldHeader::Id AdditionalFieldIds::SCAN_COUNTER;
constexpr FieldHeader::Id AdditionalFieldIds::MEASURES;
constexpr FieldHeader::Id AdditionalFieldIds::END_OF_FRAME;

FieldHeader::FieldHeader(std::istringstream& is)
{
  raw_processing::read(is, id_);
  raw_processing::read(is, length_);
  length_--;
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
  raw_processing::read(is, msg.from_theta_fixed_);
  raw_processing::read(is, msg.resolution_fixed_);

  msg.checkFixedFields();

  while (!msg.end_of_frame_)
  {
    msg.deserializeAdditionalField(is);
  }

  return msg;
}

void MonitoringFrameMsg::deserializeAdditionalField(std::istringstream& is)
{
  const FieldHeader header(is);
  const PayloadReader read_payload{ id_to_payload_reader_.at(header.id()) };
  read_payload(this, is, header.length());
}

void MonitoringFrameMsg::readScanCounter(std::istringstream& is, FieldLength length)
{
  if (length != sizeof(scan_counter_))
  {
    std::ostringstream os;
    os << "Length of scan counter field is " << length << ", but should be " << sizeof(scan_counter_) << ".";
    throw MonitoringFrameFormatError(os.str());
  }
  raw_processing::read(is, scan_counter_);
}

void MonitoringFrameMsg::readMeasures(std::istringstream& is, FieldLength length)
{
  size_t bytes_per_sample = sizeof(uint16_t);
  size_t number_of_samples = length / bytes_per_sample;

  measures_.resize(number_of_samples);

  for (unsigned i = 0; i < number_of_samples; i++)
  {
    uint16_t sample;
    raw_processing::read(is, sample);
    measures_.at(i) = sample / 1000.;  // Convert to mm
  }
}

void MonitoringFrameMsg::checkFixedFields()
{
  if (OP_CODE_MONITORING_FRAME != op_code_fixed_)
  {
    throw MonitoringFrameFormatError("Wrong Op Code!");
  }

  if (ONLINE_WORKING_MODE != working_mode_fixed_)
  {
    throw MonitoringFrameFormatError("Invalid working mode!");
  }

  if (GUI_MONITORING_TRANSACTION != transaction_type_fixed_)
  {
    throw MonitoringFrameFormatError("Invalid transaction type!");
  }

  if (MAX_SCANNER_ID < scanner_id_fixed_)
  {
    throw MonitoringFrameFormatError("Invalid Scanner id!");
  }
}

}  // namespace psen_scan_v2
