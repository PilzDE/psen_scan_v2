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
namespace monitoring_frame
{
static constexpr uint32_t DEFAULT_DEVICE_STATUS{ 0 };
static constexpr uint32_t OP_CODE_MONITORING_FRAME{ 0xCA };
static constexpr uint32_t ONLINE_WORKING_MODE{ 0x00 };
static constexpr uint32_t GUI_MONITORING_TRANSACTION{ 0x05 };
static constexpr uint16_t NUMBER_OF_BYTES_SCAN_COUNTER{ 4 };
static constexpr uint16_t NUMBER_OF_BYTES_SINGLE_MEASURE{ 2 };
class FixedFields
{
public:
  using DeviceStatus = uint32_t;
  using OpCode = uint32_t;
  using WorkingMode = uint32_t;
  using TransactionType = uint32_t;
  using FromTheta = TenthOfDegree;
  using Resolution = TenthOfDegree;

public:
  FixedFields(DeviceStatus device_status,
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
namespace additional_field
{
class Header
{
public:
  using Id = uint8_t;
  using Length = uint16_t;

public:
  Header(Id id, Length length);

public:
  Id id() const;
  Length length() const;

  static std::string idToString(Id id);

private:
  Id id_;
  Length length_;
};

enum class HeaderID : Header::Id
{
  SCAN_COUNTER = 0x02,
  DIAGNOSTICS = 0x04,
  MEASURES = 0x05,
  END_OF_FRAME = 0x09
};

Header read(std::istringstream& is, const std::size_t& max_num_bytes);
}  // namespace additional_field

Message deserialize(const MaxSizeRawData& data, const std::size_t& num_bytes);
FixedFields readFixedFields(std::istringstream& is);
std::vector<diagnostic::Message> deserializeDiagnosticMessages(std::istringstream& is);

class FormatError : public std::runtime_error
{
public:
  FormatError(const std::string& msg = "Error while decoding laser scanner measurement data");
};

class FormatErrorScanCounterUnexpectedSize : public FormatError
{
public:
  FormatErrorScanCounterUnexpectedSize(const std::string& msg);
};

inline FormatError::FormatError(const std::string& msg) : std::runtime_error(msg)
{
}

inline FormatErrorScanCounterUnexpectedSize::FormatErrorScanCounterUnexpectedSize(const std::string& msg)
  : FormatError(msg)
{
}

inline additional_field::Header::Id additional_field::Header::id() const
{
  return id_;
}

inline additional_field::Header::Length additional_field::Header::length() const
{
  return length_;
}

inline FixedFields::DeviceStatus FixedFields::device_status() const
{
  return device_status_;
}

inline FixedFields::OpCode FixedFields::op_code() const
{
  return op_code_;
}

inline FixedFields::WorkingMode FixedFields::working_mode() const
{
  return working_mode_;
}

inline FixedFields::TransactionType FixedFields::transaction_type() const
{
  return transaction_type_;
}

inline ScannerId FixedFields::scanner_id() const
{
  return scanner_id_;
}

inline FixedFields::FromTheta FixedFields::from_theta() const
{
  return from_theta_;
}

inline FixedFields::Resolution FixedFields::resolution() const
{
  return resolution_;
}

}  // namespace monitoring_frame
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_DESERIALIZATION_H
