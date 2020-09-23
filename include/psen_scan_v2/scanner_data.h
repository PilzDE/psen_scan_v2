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

#ifndef PSEN_SCAN_V2_SCANNER_DATA_H
#define PSEN_SCAN_V2_SCANNER_DATA_H

namespace psen_scan_v2
{
//! @brief Number of samples for complete scan for master scanner.
constexpr uint16_t NUMBER_OF_SAMPLES_FULL_SCAN_MASTER{ 2750 };

//! @brief Time per scan (in seconds.)
constexpr double SCAN_TIME{ 0.03 };
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_DATA_H
