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

using FieldId = FieldHeader::Id;
using FieldLength = FieldHeader::Length;
struct AdditionalFieldIds
{
  static constexpr FieldHeader::Id SCAN_COUNTER{ 0x02 };
  static constexpr FieldHeader::Id MEASURES{ 0x05 };
  static constexpr FieldHeader::Id END_OF_FRAME{ 0x09 };
  static constexpr FieldHeader::Id DIAGNOSTICS{ 0x04 };
};

MonitoringFrameMsg deserialize_monitoring_frame(const MaxSizeRawData& data, const std::size_t& num_bytes);
FieldHeader readFieldHeader(std::istringstream& is, const std::size_t& max_num_bytes);
void checkFixedFields(MonitoringFrameMsg& msg);

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

inline FieldHeader::Id FieldHeader::id() const
{
  return id_;
}

inline FieldHeader::Length FieldHeader::length() const
{
  return length_;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_DESERIALIZATION_H
