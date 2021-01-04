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
#include <boost/optional.hpp>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/diagnostics.h"
#include "psen_scan_v2/tenth_of_degree.h"
#include "psen_scan_v2/scanner_ids.h"

namespace psen_scan_v2
{
namespace monitoring_frame
{
static constexpr uint8_t MAX_SCANNER_ID{ VALID_SCANNER_IDS.size() - 1 };

/**
 * @brief Exception thrown if scan_counter was missing during deserialization of a monitoring_frame::Message.
 */
class ScanCounterMissing : public std::runtime_error
{
public:
  ScanCounterMissing(const std::string& msg = "Scan counter not set!");
};

/**
 * @brief Higher level data type representing a single monitoring frame.
 *
 * It contains all information deserialized from a single monitoring frame sent by the scanner hardware.
 *
 * @see monitoring_frame
 */
class Message
{
public:
  Message() = default;
  Message(const TenthOfDegree& from_theta,
          const TenthOfDegree& resolution,
          const uint32_t scan_counter,
          const std::vector<double>& measurements)
    : from_theta_(from_theta)
    , resolution_(resolution)
    , scan_counter_(scan_counter)
    , measurements_(measurements)
    , diagnostic_data_enabled_(false){

    };

  Message(const TenthOfDegree& from_theta,
          const TenthOfDegree& resolution,
          const uint32_t scan_counter,
          const std::vector<double>& measurements,
          const std::vector<double>& intensities,
          const std::vector<monitoring_frame::diagnostic::Message>& diagnostic_messages)
    : from_theta_(from_theta)
    , resolution_(resolution)
    , scan_counter_(scan_counter)
    , measurements_(measurements)
    , intensities_(intensities)
    , diagnostic_messages_(diagnostic_messages)
    , diagnostic_data_enabled_(true){

    };

public:
  TenthOfDegree fromTheta() const;
  TenthOfDegree resolution() const;
  uint32_t scanCounter() const;
  const std::vector<double>& measurements() const;
  const std::vector<double>& intensities() const;
  std::vector<monitoring_frame::diagnostic::Message> diagnosticMessages() const;
  bool operator==(const monitoring_frame::Message& rhs) const;

private:
  ScannerId scanner_id_{ ScannerId::master };
  TenthOfDegree from_theta_{ 0 };
  TenthOfDegree resolution_{ 0 };

  boost::optional<uint32_t> scan_counter_;
  std::vector<double> measurements_;
  std::vector<double> intensities_;
  std::vector<monitoring_frame::diagnostic::Message> diagnostic_messages_;
  bool diagnostic_data_enabled_{ false };

public:
  friend RawData serialize(const monitoring_frame::Message& frame);
  friend monitoring_frame::Message deserialize(const RawData& data, const std::size_t& num_bytes);
};

inline ScanCounterMissing::ScanCounterMissing(const std::string& msg) : std::runtime_error(msg)
{
}

std::ostream& operator<<(std::ostream& os, const psen_scan_v2::monitoring_frame::Message& msg);

}  // namespace monitoring_frame
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_MSG_H
