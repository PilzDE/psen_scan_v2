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
#ifndef PSEN_SCAN_V2_MONITORING_FRAME_DESERIALIZATION_H
#define PSEN_SCAN_V2_MONITORING_FRAME_DESERIALIZATION_H

#include <ostream>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/raw_processing.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/logging.h"

namespace psen_scan_v2
{
static constexpr uint32_t DEFAULT_DEVICE_STATUS{ 0 };
static constexpr uint32_t OP_CODE_MONITORING_FRAME{ 0xCA };
static constexpr uint32_t ONLINE_WORKING_MODE{ 0x00 };
static constexpr uint32_t GUI_MONITORING_TRANSACTION{ 0x05 };
static constexpr uint16_t NUMBER_OF_BYTES_SCAN_COUNTER{ 4 };
static constexpr uint16_t NUMBER_OF_BYTES_SINGLE_MEASURE{ 2 };
static constexpr uint16_t NUMBER_OF_BYTES_SINGLE_INTENSITY{ 2 };
class MonitoringFrameAdditionalFieldHeader
{
public:
  using Id = uint8_t;
  using Length = uint16_t;

public:
  MonitoringFrameAdditionalFieldHeader(Id id, Length length);

public:
  Id id() const;
  Length length() const;

  static std::string idToString(Id id);

private:
  Id id_;
  Length length_;
};

class MonitoringFrameFixedFields
{
public:
  using DeviceStatus = uint32_t;
  using OpCode = uint32_t;
  using WorkingMode = uint32_t;
  using TransactionType = uint32_t;
  using FromTheta = TenthOfDegree;
  using Resolution = TenthOfDegree;

public:
  MonitoringFrameFixedFields(DeviceStatus device_status,
                             OpCode op_code,
                             WorkingMode working_mode,
                             TransactionType transaction_type,
                             ScannerId scanner_id,
                             FromTheta from_theta,
                             Resolution resolution);

public:
  DeviceStatus device_status() const;
  OpCode op_code() const;
  WorkingMode working_mode() const;
  TransactionType transaction_type() const;
  ScannerId scanner_id() const;
  FromTheta from_theta() const;
  Resolution resolution() const;

private:
  DeviceStatus device_status_;
  OpCode op_code_;
  WorkingMode working_mode_;
  TransactionType transaction_type_;
  ScannerId scanner_id_;
  FromTheta from_theta_;
  Resolution resolution_;
};
namespace monitoring_frame_additional_field_header_ids
{
enum class HeaderID : MonitoringFrameAdditionalFieldHeader::Id
{
  SCAN_COUNTER = 0x02,
  DIAGNOSTICS = 0x04,
  MEASURES = 0x05,
  INTENSITIES = 0x06,
  END_OF_FRAME = 0x09
};
};  // namespace monitoring_frame_additional_field_header_ids

MonitoringFrameMsg deserializeMonitoringFrame(const MaxSizeRawData& data, const std::size_t& num_bytes);
MonitoringFrameFixedFields readHeader(std::istringstream& is);
MonitoringFrameAdditionalFieldHeader readFieldHeader(std::istringstream& is, const std::size_t& max_num_bytes);
std::vector<MonitoringFrameDiagnosticMessage> deserializeDiagnosticMessages(std::istringstream& is);

class MonitoringFrameFormatError : public std::runtime_error
{
public:
  MonitoringFrameFormatError(const std::string& msg = "Error while decoding laser scanner measurement data");
};

class MonitoringFrameFormatErrorScanCounterUnexpectedSize : public MonitoringFrameFormatError
{
public:
  MonitoringFrameFormatErrorScanCounterUnexpectedSize(const std::string& msg);
};

inline MonitoringFrameFormatError::MonitoringFrameFormatError(const std::string& msg) : std::runtime_error(msg)
{
}

inline MonitoringFrameFormatErrorScanCounterUnexpectedSize::MonitoringFrameFormatErrorScanCounterUnexpectedSize(
    const std::string& msg)
  : MonitoringFrameFormatError(msg)
{
}

inline MonitoringFrameAdditionalFieldHeader::Id MonitoringFrameAdditionalFieldHeader::id() const
{
  return id_;
}

inline MonitoringFrameAdditionalFieldHeader::Length MonitoringFrameAdditionalFieldHeader::length() const
{
  return length_;
}

inline MonitoringFrameFixedFields::DeviceStatus MonitoringFrameFixedFields::device_status() const
{
  return device_status_;
}

inline MonitoringFrameFixedFields::OpCode MonitoringFrameFixedFields::op_code() const
{
  return op_code_;
}

inline MonitoringFrameFixedFields::WorkingMode MonitoringFrameFixedFields::working_mode() const
{
  return working_mode_;
}

inline MonitoringFrameFixedFields::TransactionType MonitoringFrameFixedFields::transaction_type() const
{
  return transaction_type_;
}

inline ScannerId MonitoringFrameFixedFields::scanner_id() const
{
  return scanner_id_;
}

inline MonitoringFrameFixedFields::FromTheta MonitoringFrameFixedFields::from_theta() const
{
  return from_theta_;
}

inline MonitoringFrameFixedFields::Resolution MonitoringFrameFixedFields::resolution() const
{
  return resolution_;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_DESERIALIZATION_H
