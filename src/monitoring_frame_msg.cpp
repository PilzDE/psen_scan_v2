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
#include <cassert>
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
constexpr uint8_t MonitoringFrameIds::SCAN_COUNTER;
constexpr uint8_t MonitoringFrameIds::MEASURES;
constexpr uint8_t MonitoringFrameIds::END_OF_FRAME;

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

  while (!msg.end_of_frame_)
  {
    msg.deserializeAdditionalField(is);
  }

  return msg;
}

void MonitoringFrameMsg::deserializeAdditionalField(std::istringstream& is)
{
  uint8_t id;
  raw_processing::read(is, id);
  const SingleFieldReader read_single_field{ id_to_field_reader_.at(id) };
  read_single_field(is);
}

void ScanCounterField::readLengthAndPayload(std::istringstream& is, uint32_t& scan_counter)
{
  uint16_t length;
  raw_processing::read(is, length);
  if (length != sizeof(scan_counter))
  {
    std::ostringstream os;
    os << "Length of scan counter field is " << length << ", but should be " << sizeof(scan_counter) << ".";
    throw MonitoringFrameFormatError(os.str());
  }
  raw_processing::read(is, scan_counter);
}

void MeasuresField::readLengthAndPayload(std::istringstream& is, std::vector<uint16_t>& measures)
{
  uint16_t length;
  raw_processing::read(is, length);

  size_t bytes_per_sample = sizeof(uint16_t);
  size_t number_of_samples = length / bytes_per_sample;

  measures.resize(number_of_samples);

  for (unsigned i = 0; i < number_of_samples; i++)
  {
      uint16_t sample;
      raw_processing::read(is, sample);
      measures.at(i) = sample;
  }

  assert(is.eof());
  // TODO get this to work or remove the comment
  //std::copy_n(std::istream_iterator<uint16_t>(is), number_of_samples, measures.begin());
}

void EndOfFrameField::setEndOfFrameMemberToTrue(std::istringstream& is, bool& end_of_frame)
{
  end_of_frame = true;
}

}  // namespace psen_scan_v2
