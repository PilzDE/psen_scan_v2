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

#ifndef PSEN_SCAN_V2_MONITORING_FRAME_MSG_H
#define PSEN_SCAN_V2_MONITORING_FRAME_MSG_H

#include <cstdint>
#include <functional>
#include <map>
#include <sstream>
#include <vector>

#include "psen_scan_v2/raw_scanner_data.h"

namespace psen_scan_v2
{
struct ScanCounterField
{
  static void readLengthAndPayload(std::istringstream& is, uint32_t& scan_counter);
};

struct MeasuresField
{
  static void readLengthAndPayload(std::istringstream& is, std::vector<uint16_t>& measures);
};

struct EndOfFrameField
{
  static void setEndOfFrameMemberToTrue(std::istringstream& is, bool& end_of_frame);
};

struct MonitoringFrameIds
{
  static constexpr uint8_t SCAN_COUNTER{ 0x02 };
  static constexpr uint8_t MEASURES{ 0x05 };
  static constexpr uint8_t END_OF_FRAME{ 0x09 };
};

class MonitoringFrameMsg
{
public:
  static MonitoringFrameMsg fromRawData(const MaxSizeRawData& data);

public:
  uint16_t fromTheta() const;
  uint16_t resolution() const;
  std::vector<uint16_t> measures() const;

private:
  void deserializeAdditionalField(std::istringstream& is);

private:
  uint32_t device_status_fixed_{ 0 };
  uint32_t op_code_fixed_{ 0 };
  uint32_t working_mode_fixed_{ 0 };
  uint32_t transaction_type_fixed_{ 0 };
  uint8_t scanner_id_fixed_{ 0 };
  uint16_t from_theta_fixed_{ 0 };
  uint16_t resolution_fixed_{ 0 };

  // Additional information
  uint32_t scan_counter_{ 0 };
  std::vector<uint16_t> measures_;

  bool end_of_frame_{ false };

private:
  using SingleFieldReader = std::function<void(std::istringstream&)>;
  std::map<uint8_t, SingleFieldReader> id_to_field_reader_{
    { MonitoringFrameIds::SCAN_COUNTER, std::bind(ScanCounterField::readLengthAndPayload, std::placeholders::_1, scan_counter_) },
    { MonitoringFrameIds::MEASURES, std::bind(MeasuresField::readLengthAndPayload, std::placeholders::_1, measures_) },
    { MonitoringFrameIds::END_OF_FRAME, std::bind(EndOfFrameField::setEndOfFrameMemberToTrue, std::placeholders::_1, end_of_frame_) }
  };
};

inline uint16_t MonitoringFrameMsg::fromTheta() const
{
  return from_theta_fixed_;
}

inline uint16_t MonitoringFrameMsg::resolution() const
{
  return resolution_fixed_;
}

inline std::vector<uint16_t> MonitoringFrameMsg::measures() const
{
  return measures_;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_MSG_H
