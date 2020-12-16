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
#ifndef PSEN_SCAN_V2_STOP_REQUEST_SERIALIZATION_H
#define PSEN_SCAN_V2_STOP_REQUEST_SERIALIZATION_H

#include <cstdint>
#include <array>

#include "psen_scan_v2/raw_scanner_data.h"

namespace psen_scan_v2
{
/**
 * @brief Contains all things needed to define and implement a scanner stop request.
 */
namespace stop_request
{
static constexpr std::size_t NUM_RESERVED_FIELDS{ 12 };
static constexpr std::array<uint8_t, NUM_RESERVED_FIELDS> RESERVED{};
static const uint32_t OPCODE{ htole32(0x36) };

RawData serialize();
}  // namespace stop_request
}  // namespace psen_scan_v2
#endif
