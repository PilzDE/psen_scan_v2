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

#include <ostream>

#include "psen_scan_v2/monitoring_frame_serialization.h"
#include "psen_scan_v2/raw_processing.h"

namespace psen_scan_v2
{
DynamicSizeRawData serialize(MonitoringFrameMsg& frame)
{
  std::ostringstream os;

  raw_processing::write(os, frame.device_status_fixed_);
  raw_processing::write(os, frame.op_code_fixed_);
  raw_processing::write(os, frame.working_mode_fixed_);
  raw_processing::write(os, frame.transaction_type_fixed_);
  raw_processing::write(os, frame.scanner_id_fixed_);
  raw_processing::write(os, frame.from_theta_fixed_);
  raw_processing::write(os, frame.resolution_fixed_);

  uint8_t scan_counter_header_id = AdditionalFieldIds::SCAN_COUNTER;
  uint16_t scan_counter_header_length = sizeof(frame.scan_counter_) + 1;
  uint32_t scan_counter_header_payload = frame.scan_counter_;
  raw_processing::write(os, scan_counter_header_id);
  raw_processing::write(os, scan_counter_header_length);
  raw_processing::write(os, scan_counter_header_payload);

  uint8_t measures_header_id = AdditionalFieldIds::MEASURES;
  uint16_t measures_header_length = frame.measures_.size() * NUMBER_OF_BYTES_SINGLE_MEASURE + 1;
  raw_processing::write(os, measures_header_id);
  raw_processing::write(os, measures_header_length);
  for (size_t i = 0; i < frame.measures_.size(); ++i)
  {
    uint16_t measure = static_cast<uint16_t>(std::round(frame.measures_.at(i) * 1000));
    raw_processing::write(os, measure);
  }

  uint8_t end_of_frame_header_id = AdditionalFieldIds::END_OF_FRAME;
  raw_processing::write(os, end_of_frame_header_id);

  uint8_t unknown_data_at_the_end_of_frame = 0;
  raw_processing::write(os, unknown_data_at_the_end_of_frame);
  raw_processing::write(os, unknown_data_at_the_end_of_frame);
  raw_processing::write(os, unknown_data_at_the_end_of_frame);

  return raw_processing::serialize<DynamicSizeRawData>(os);
}
}  // namespace psen_scan_v2