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
#include <string>
#include <vector>
#include <array>
#include <bitset>

#include <fmt/core.h>
#include <fmt/ranges.h>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/tenth_of_degree.h"
#include "psen_scan_v2/diagnostics.h"

namespace psen_scan_v2
{
class MonitoringFrameDiagnosticMessage;

static constexpr uint32_t OP_CODE_MONITORING_FRAME{ 0xCA };
static constexpr uint32_t ONLINE_WORKING_MODE{ 0x00 };
static constexpr uint32_t GUI_MONITORING_TRANSACTION{ 0x05 };
static constexpr uint8_t MAX_SCANNER_ID{ 0x03 };

static constexpr uint16_t NUMBER_OF_BYTES_SCAN_COUNTER{ 4 };
static constexpr uint16_t NUMBER_OF_BYTES_SINGLE_MEASURE{ 2 };

enum class ScannerId : uint8_t
{
  MASTER = 0,
  SLAVE0 = 1,
  SLAVE1 = 2,
  SLAVE2 = 3
};

static const std::array<ScannerId, 4> SCANNER_IDS{ ScannerId::MASTER,
                                                   ScannerId::SLAVE0,
                                                   ScannerId::SLAVE1,
                                                   ScannerId::SLAVE2 };

class MonitoringFrameMsg
{
public:
  MonitoringFrameMsg() = default;
  MonitoringFrameMsg(const TenthOfDegree& from_theta,
                     const TenthOfDegree& resolution,
                     const uint32_t scan_counter,
                     const std::vector<double> measures)
    : from_theta_fixed_(from_theta)
    , resolution_fixed_(resolution)
    , scan_counter_(scan_counter)
    , measures_(measures){

    };

public:
  TenthOfDegree fromTheta() const;
  TenthOfDegree resolution() const;
  uint32_t scanCounter() const;
  std::vector<double> measures() const;
  std::vector<MonitoringFrameDiagnosticMessage> diagnostic_messages() const;
  bool operator==(const MonitoringFrameMsg& rhs) const;

  friend std::ostream& operator<<(std::ostream& os, const MonitoringFrameMsg& msg)
  {
    os << fmt::format("MonitoringFrameMsg(fromTheta = {} deg, resolution = {} deg, scanCounter = "
                      "{}, measures = {})",
                      msg.fromTheta().value() / 10.,
                      msg.resolution().value() / 10.,
                      msg.scanCounter(),
                      msg.measures_);
    return os;
  }

private:
  uint32_t device_status_fixed_{ 0 };
  uint32_t op_code_fixed_{ OP_CODE_MONITORING_FRAME };
  uint32_t working_mode_fixed_{ 0 };
  uint32_t transaction_type_fixed_{ GUI_MONITORING_TRANSACTION };
  ScannerId scanner_id_{ ScannerId::MASTER };
  TenthOfDegree from_theta_fixed_{ 0 };
  TenthOfDegree resolution_fixed_{ 0 };

  uint32_t scan_counter_{ 0 };
  std::vector<double> measures_;
  std::vector<MonitoringFrameDiagnosticMessage> diagnostic_messages_;

public:
  friend DynamicSizeRawData serialize(MonitoringFrameMsg& frame);
  friend MonitoringFrameMsg deserialize_monitoring_frame(const MaxSizeRawData& data, const std::size_t& num_bytes);
  friend void checkFixedFields(MonitoringFrameMsg& msg);
};
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_MSG_H
