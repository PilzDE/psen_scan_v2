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

#ifndef PSEN_SCAN_V2_SCANNER_CONSTANTS_H
#define PSEN_SCAN_V2_SCANNER_CONSTANTS_H

#include <cstdint>

#include "psen_scan_v2/angle_conversions.h"

namespace psen_scan_v2
{
static constexpr double MASTER_RESOLUTION_RAD{ degreeToRadian(0.1) };
static constexpr uint16_t NUMBER_OF_SAMPLES_FULL_SCAN_MASTER{ 2750 };

static constexpr uint32_t MAX_SCANNER_ID{ 0x03 };

static constexpr double TIME_PER_SCAN_IN_S{ 0.03 };

static constexpr double RANGE_MIN_IN_M{ 0. };
static constexpr double RANGE_MAX_IN_M{ 10. };

static constexpr double DEFAULT_X_AXIS_ROTATION(degreeToRadian(137.5));

namespace start_request_constants
{
static constexpr std::size_t START_REQUEST_SIZE{ 58 };  // See protocol
const uint32_t OPCODE{ htole32(0x35) };
static constexpr std::size_t OFFSET_CRC{ 0x00 };
static constexpr std::size_t OFFSET_SEQ_NUMBER{ 0x04 };
static constexpr std::size_t OFFSET_RESERVED{ 0x08 };
static constexpr std::size_t OFFSET_OPCODE{ 0x10 };
static constexpr std::size_t OFFSET_IP{ 0x14 };
static constexpr std::size_t OFFSET_UDP_PORT{ 0x18 };
static constexpr std::size_t OFFSET_DEVICE_ENABLED{ 0x1A };
static constexpr std::size_t OFFSET_INTENSITIES_ENABLED{ 0x1B };
static constexpr std::size_t OFFSET_POINT_IN_SAFETY_ENABLED{ 0x1C };
static constexpr std::size_t OFFSET_ACTIVE_ZONE_SET_ENABLED{ 0x1D };
static constexpr std::size_t OFFSET_IO_PIN_ENABLED{ 0x1E };
static constexpr std::size_t OFFSET_SCAN_COUNTER_ENABLED{ 0x1F };
static constexpr std::size_t OFFSET_SPEED_ENCODER_ENABLED{ 0x20 };
static constexpr std::size_t OFFSET_DIAGNOSTICS_ENABLED{ 0x21 };
static constexpr std::size_t OFFSET_MASTER_START_ANGLE{ 0x22 };
static constexpr std::size_t OFFSET_MASTER_END_ANGLE{ 0x24 };
static constexpr std::size_t OFFSET_MASTER_ANGLE_RESOLUTION{ 0x26 };
static constexpr std::size_t OFFSET_SLAVE_ONE_START_ANGLE{ 0x28 };
static constexpr std::size_t OFFSET_SLAVE_ONE_END_ANGLE{ 0x2A };
static constexpr std::size_t OFFSET_SLAVE_ONE_ANGLE_RESOLUTION{ 0x2C };
static constexpr std::size_t OFFSET_SLAVE_TWO_START_ANGLE{ 0x2E };
static constexpr std::size_t OFFSET_SLAVE_TWO_END_ANGLE{ 0x30 };
static constexpr std::size_t OFFSET_SLAVE_TWO_ANGLE_RESOLUTION{ 0x32 };
static constexpr std::size_t OFFSET_SLAVE_THREE_START_ANGLE{ 0x34 };
static constexpr std::size_t OFFSET_SLAVE_THREE_END_ANGLE{ 0x36 };
static constexpr std::size_t OFFSET_SLAVE_THREE_ANGLE_RESOLUTION{ 0x38 };
}  // namespace start_request_constants

namespace stop_request_constants
{
static constexpr std::size_t STOP_REQUEST_SIZE{ 20 };
static constexpr std::size_t NUM_RESERVED_FIELDS{ 12 };
static constexpr std::size_t OFFSET_CRC{ 0x00 };
static constexpr std::size_t OFFSET_RESERVED{ 0x04 };
const uint32_t OPCODE{ htole32(0x36) };
}  // namespace stop_request_constants

namespace monitoring_frame_constants
{
static constexpr std::size_t MAX_LENGTH_ADDITIONAL_MONITORING_FRAME_FIELD{ 65487 };

static constexpr uint32_t OPCODE{ 0xCA };
static constexpr uint32_t ONLINE_WORKING_MODE{ 0x00 };
static constexpr uint32_t GUI_MONITORING_TRANSACTION{ 0x05 };

static constexpr std::size_t OFFSET_OPCODE{ 0x04 };
static constexpr std::size_t OFFSET_WORKING_MODE{ 0x08 };
static constexpr std::size_t OFFSET_TRANSACTION_TYPE{ 0x0C };
static constexpr std::size_t OFFSET_SCANNER_ID{ 0x10 };

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

inline FieldHeader::FieldHeader(Id id, Length length) : id_(id), length_(length)
{
}

inline std::string FieldHeader::idToString(Id id)
{
  std::ostringstream os;
  os << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(id);
  return os.str();
}

struct AdditionalFieldIds
{
  static constexpr FieldHeader::Id SCAN_COUNTER{ 0x02 };
  static constexpr FieldHeader::Id MEASURES{ 0x05 };
  static constexpr FieldHeader::Id END_OF_FRAME{ 0x09 };
};
}  // namespace monitoring_frame_constants
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_CONSTANTS_H
