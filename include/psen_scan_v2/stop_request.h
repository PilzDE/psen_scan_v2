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
#ifndef PSEN_SCAN_STOP_REQUEST_H
#define PSEN_SCAN_STOP_REQUEST_H

#include <cstdint>
#include <iostream>
#include <array>

#include "psen_scan_v2/raw_scanner_data.h"

namespace psen_scan_v2
{
/**
 * @brief Higher level data type representing a scanner stop request.
 *
 * @note Unless otherwise indicated the byte order is little endian.
 *
 */
class StopRequest
{
public:
  static constexpr std::size_t STOP_REQUEST_SIZE{ 20 };
  static constexpr std::size_t NUM_RESERVED_FIELDS{ 12 };

public:
 DynamicSizeRawData serialize() const;

private:
  std::ostringstream& processMember(std::ostringstream& os) const;
  uint32_t calcCrc() const;

private:
  const std::array<uint8_t, NUM_RESERVED_FIELDS> RESERVED_{};
  const uint32_t OPCODE_{ htole32(0x36) };
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_STOP_REQUEST_H
