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
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <bitset>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/diagnostics.h"
#include "psen_scan_v2/tenth_of_degree.h"
#include "psen_scan_v2/scanner_ids.h"

namespace psen_scan_v2
{
static constexpr uint8_t MAX_SCANNER_ID{ VALID_SCANNER_IDS.size() - 1 };

class MonitoringFrameMsg
{
public:
  MonitoringFrameMsg() = default;
  MonitoringFrameMsg(const TenthOfDegree& from_theta,
                     const TenthOfDegree& resolution,
                     const uint32_t scan_counter,
                     const std::vector<double>& measures)
    : from_theta_(from_theta)
    , resolution_(resolution)
    , scan_counter_(scan_counter)
    , measures_(measures)
    , diagnostic_data_enabled_(false){

    };

  MonitoringFrameMsg(const TenthOfDegree& from_theta,
                     const TenthOfDegree& resolution,
                     const uint32_t scan_counter,
                     const std::vector<double>& measures,
                     const std::vector<MonitoringFrameDiagnosticMessage>& diagnostic_messages)
    : from_theta_(from_theta)
    , resolution_(resolution)
    , scan_counter_(scan_counter)
    , measures_(measures)
    , diagnostic_messages_(diagnostic_messages)
    , diagnostic_data_enabled_(true){

    };

public:
  TenthOfDegree fromTheta() const;
  TenthOfDegree resolution() const;
  uint32_t scanCounter() const;
  std::vector<double> measures() const;
  std::vector<MonitoringFrameDiagnosticMessage> diagnosticMessages() const;
  bool operator==(const MonitoringFrameMsg& rhs) const;

private:
  ScannerId scanner_id_{ ScannerId::MASTER };
  TenthOfDegree from_theta_{ 0 };
  TenthOfDegree resolution_{ 0 };

  uint32_t scan_counter_{ 0 };
  std::vector<double> measures_;
  std::vector<MonitoringFrameDiagnosticMessage> diagnostic_messages_;
  bool diagnostic_data_enabled_{ false };

public:
  friend DynamicSizeRawData serialize(const MonitoringFrameMsg& frame);
  friend MonitoringFrameMsg deserializeMonitoringFrame(const MaxSizeRawData& data, const std::size_t& num_bytes);
};

}  // namespace psen_scan_v2

std::ostream& operator<<(std::ostream& os, const psen_scan_v2::MonitoringFrameMsg& msg);

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_MSG_H
