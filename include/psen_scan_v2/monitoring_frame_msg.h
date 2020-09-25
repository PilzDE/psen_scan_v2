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
#include "psen_scan_v2/scanner_constants.h"

using namespace psen_scan_v2::monitoring_frame_constants;
namespace psen_scan_v2
{
class MonitoringFrameMsg
{
public:
  static MonitoringFrameMsg fromRawData(const MaxSizeRawData& data);

public:
  double fromTheta() const;
  double resolution() const;
  uint32_t scanCounter() const;
  std::vector<double> measures() const;

private:
  using FieldId = FieldHeader::Id;
  using FieldLength = FieldHeader::Length;

private:
  static void readAngle(std::istringstream& is, double& angle);

  static FieldHeader readFieldHeader(std::istringstream& is);
  static void readScanCounter(std::istringstream& is, uint32_t& scan_counter, const FieldLength length);
  static void readMeasures(std::istringstream& is, std::vector<double>& measures, const FieldLength length);

  void checkFixedFields();

private:
  FRIEND_TEST(FieldHeaderTest, testReadSuccess);
  FRIEND_TEST(FieldHeaderTest, testReadHeaderTooShortFailure);
  FRIEND_TEST(MonitoringFrameMsgTest, testReadScanCounterSuccess);
  FRIEND_TEST(MonitoringFrameMsgTest, testReadScanCounterInvalidLengthFailure);
  FRIEND_TEST(MonitoringFrameMsgTest, testReadScanCounterMissingPayloadFailure);
  FRIEND_TEST(MonitoringFrameMsgTest, testReadMeasuresSuccess);
  FRIEND_TEST(MonitoringFrameMsgTest, testReadMeasuresMissingPayloadFailure);
  FRIEND_TEST(MonitoringFrameMsgTest, testReadMeasuresTooMuchMeasures);
  FRIEND_TEST(MonitoringFrameMsgTest, testReadMeasuresTooFewMeasures);
  FRIEND_TEST(MonitoringFrameMsgTest, testSetEndOfFrame);
  FRIEND_TEST(MonitoringFrameMsgTest, testSetEndOfFrameIgnoreInvalidLength);

private:
  uint32_t device_status_fixed_{ 0 };
  uint32_t op_code_fixed_{ 0 };
  uint32_t working_mode_fixed_{ 0 };
  uint32_t transaction_type_fixed_{ 0 };
  uint8_t scanner_id_fixed_{ 0 };
  double from_theta_fixed_{ 0 };
  double resolution_fixed_{ 0 };

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

inline double MonitoringFrameMsg::fromTheta() const
{
  return from_theta_fixed_;
}

inline double MonitoringFrameMsg::resolution() const
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

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_MSG_H
