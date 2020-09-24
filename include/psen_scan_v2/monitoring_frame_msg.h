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

#include <gtest/gtest_prod.h>

#include "psen_scan_v2/raw_scanner_data.h"

namespace psen_scan_v2
{
static constexpr uint32_t OP_CODE_MONITORING_FRAME{ 0xCA };
static constexpr uint32_t ONLINE_WORKING_MODE{ 0x00 };
static constexpr uint32_t GUI_MONITORING_TRANSACTION{ 0x05 };
static constexpr uint32_t MAX_SCANNER_ID{ 0x03 };

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
  static MonitoringFrameMsg fromRawData(const MaxSizeRawData& data);

public:
  uint16_t fromTheta() const;
  uint16_t resolution() const;
  uint32_t scanCounter() const;
  std::vector<double> measures() const;

private:
  using FieldId = FieldHeader::Id;
  using FieldLength = FieldHeader::Length;

private:
  void deserializeAdditionalField(std::istringstream& is);

  void readScanCounter(std::istringstream& is, FieldLength length);
  void readMeasures(std::istringstream& is, FieldLength length);

  void setEndOfFrame(std::istringstream& /*is*/, FieldLength /*length*/);

  void checkFixedFields();

private:
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
  uint16_t from_theta_fixed_{ 0 };
  uint16_t resolution_fixed_{ 0 };

  uint32_t scan_counter_{ 0 };
  std::vector<double> measures_;
  bool end_of_frame_{ false };

  using PayloadReader = std::function<void(MonitoringFrameMsg*, std::istringstream&, FieldLength)>;
  std::map<FieldId, PayloadReader> id_to_payload_reader_{
    { AdditionalFieldIds::SCAN_COUNTER, &MonitoringFrameMsg::readScanCounter },
    { AdditionalFieldIds::MEASURES, &MonitoringFrameMsg::readMeasures },
    { AdditionalFieldIds::END_OF_FRAME, &MonitoringFrameMsg::setEndOfFrame }
  };
};

inline FieldHeader::Id FieldHeader::id() const
{
  return id_;
}

inline FieldHeader::Length FieldHeader::length() const
{
  return length_;
}

inline uint16_t MonitoringFrameMsg::fromTheta() const
{
  return from_theta_fixed_;
}

inline uint16_t MonitoringFrameMsg::resolution() const
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

inline void MonitoringFrameMsg::setEndOfFrame(std::istringstream& /*is*/, FieldLength /*length*/)
{
  end_of_frame_ = true;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_MSG_H
