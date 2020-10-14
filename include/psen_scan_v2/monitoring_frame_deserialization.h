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

using MonitoringFrameAdditionalFieldId = MonitoringFrameAdditionalFieldHeader::Id;
using MonitoringFrameAdditionalFieldLength = MonitoringFrameAdditionalFieldHeader::Length;
struct MonitoringFrameAdditionalFieldIds
{
  static constexpr MonitoringFrameAdditionalFieldHeader::Id SCAN_COUNTER{ 0x02 };
  static constexpr MonitoringFrameAdditionalFieldHeader::Id MEASURES{ 0x05 };
  static constexpr MonitoringFrameAdditionalFieldHeader::Id END_OF_FRAME{ 0x09 };
  static constexpr MonitoringFrameAdditionalFieldHeader::Id DIAGNOSTICS{ 0x04 };
};

MonitoringFrameMsg deserialize_monitoring_frame(const MaxSizeRawData& data, const std::size_t& num_bytes);
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

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MONITORING_FRAME_DESERIALIZATION_H
