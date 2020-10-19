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

#ifndef PSEN_SCAN_V2_SCANNER_IDS_H
#define PSEN_SCAN_V2_SCANNER_IDS_H

#include <array>

namespace psen_scan_v2
{
enum class ScannerId : uint8_t
{
  MASTER = 0,
  SLAVE0 = 1,
  SLAVE1 = 2,
  SLAVE2 = 3
};

static constexpr std::array<ScannerId, 4> VALID_SCANNER_IDS{ ScannerId::MASTER,
                                                       ScannerId::SLAVE0,
                                                       ScannerId::SLAVE1,
                                                       ScannerId::SLAVE2 };

static const std::map<ScannerId, std::string> scanner_id_to_string{ { ScannerId::MASTER, "Master" },
                                                                    { ScannerId::SLAVE0, "Slave0" },
                                                                    { ScannerId::SLAVE1, "Slave1" },
                                                                    { ScannerId::SLAVE2, "Slave2" } };

}  //   namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_IDS_H
