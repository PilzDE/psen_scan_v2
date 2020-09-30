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

#include <gtest/gtest_prod.h>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/tenth_of_degree.h"

namespace psen_scan_v2
{
static constexpr uint32_t OP_CODE_MONITORING_FRAME{ 0xCA };
static constexpr uint32_t ONLINE_WORKING_MODE{ 0x00 };
static constexpr uint32_t GUI_MONITORING_TRANSACTION{ 0x05 };
static constexpr uint32_t MAX_SCANNER_ID{ 0x03 };

static constexpr uint16_t NUMBER_OF_BYTES_SCAN_COUNTER{ 4 };
static constexpr uint16_t NUMBER_OF_BYTES_SINGLE_MEASURE{ 2 };

class FieldHeader
{
public:
  using Id = uint8_t;
  using Length = uint16_t;

public:
  FieldHeader(Id id, Length length);

public:
  Id id() const;
  Length length() const;

  static std::string idToString(Id id);

private:
  Id id_;
  Length length_;
};

struct AdditionalFieldIds
{
  static constexpr FieldHeader::Id SCAN_COUNTER{ 0x02 };
  static constexpr FieldHeader::Id MEASURES{ 0x05 };
  static constexpr FieldHeader::Id END_OF_FRAME{ 0x09 };
};

class MonitoringFrameMsg
{
public:
  class MonitoringFrameFormatError : public std::runtime_error
  {
  public:
    MonitoringFrameFormatError(const std::string& msg = "Error while decoding laser scanner measurement data");
  };

  class MonitoringFrameFormatErrorScanCounterUnexpectedSize : public MonitoringFrameFormatError
  {
  public:
    MonitoringFrameFormatErrorScanCounterUnexpectedSize(const std::string& msg) : MonitoringFrameFormatError(msg)
    {
    }
  };

public:
  static MonitoringFrameMsg fromRawData(const MaxSizeRawData& data, const std::size_t& num_bytes);

public:
  TenthOfDegree fromTheta() const;
  TenthOfDegree resolution() const;
  uint32_t scanCounter() const;
  std::vector<double> measures() const;

private:
  using FieldId = FieldHeader::Id;
  using FieldLength = FieldHeader::Length;

private:
  static void readAngle(std::istringstream& is, double& angle);

  static FieldHeader readFieldHeader(std::istringstream& is, const std::size_t& max_num_bytes);
  static void readScanCounter(std::istringstream& is, uint32_t& scan_counter, const FieldLength length);
  static void readMeasures(std::istringstream& is, std::vector<double>& measures, const FieldLength length);

  void checkFixedFields();

private:
  FRIEND_TEST(FieldHeaderTest, testReadSuccess);
  FRIEND_TEST(FieldHeaderTest, testReadHeaderTooShortFailure);

private:
  uint32_t device_status_fixed_{ 0 };
  uint32_t op_code_fixed_{ 0 };
  uint32_t working_mode_fixed_{ 0 };
  uint32_t transaction_type_fixed_{ 0 };
  uint8_t scanner_id_fixed_{ 0 };
  TenthOfDegree from_theta_fixed_{ 0 };
  TenthOfDegree resolution_fixed_{ 0 };

  uint32_t scan_counter_{ 0 };
  std::vector<double> measures_;
};

inline FieldHeader::Id FieldHeader::id() const
{
  return id_;
}

inline FieldHeader::Length FieldHeader::length() const
{
  return length_;
}

inline TenthOfDegree MonitoringFrameMsg::fromTheta() const
{
  return from_theta_fixed_;
}

inline TenthOfDegree MonitoringFrameMsg::resolution() const
{
  return resolution_fixed_;
}

inline uint32_t MonitoringFrameMsg::scanCounter() const
{
  return scan_counter_;
}

inline std::vector<double> MonitoringFrameMsg::measures() const
{
  return measures_;
}

inline MonitoringFrameMsg::MonitoringFrameFormatError::MonitoringFrameFormatError(const std::string& msg)
  : std::runtime_error(msg)
{
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_MSG_H
