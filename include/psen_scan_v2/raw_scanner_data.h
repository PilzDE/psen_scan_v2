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
#ifndef PSEN_SCAN_V2_RAW_SCANNER_DATA_H
#define PSEN_SCAN_V2_RAW_SCANNER_DATA_H

#include <array>

namespace psen_scan_v2
{
template <std::size_t NumberOfBytes>
using RawDataContainer = std::array<char, NumberOfBytes>;

static constexpr std::size_t MAX_UDP_DATA_SIZE_IN_BYTES{ 65507 };
using RawScannerData = RawDataContainer<MAX_UDP_DATA_SIZE_IN_BYTES>;

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_RAW_SCANNER_DATA_H
