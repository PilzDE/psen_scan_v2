// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_SCANNER_IDS_H
#define PSEN_SCAN_V2_STANDALONE_SCANNER_IDS_H

#include <array>
#include <map>
#include <string>

namespace psen_scan_v2_standalone
{
namespace configuration
{
enum class ScannerId : uint8_t
{
  master = 0,
  slave0 = 1,
  slave1 = 2,
  slave2 = 3
};

static constexpr std::array<ScannerId, 4> VALID_SCANNER_IDS{ ScannerId::master,
                                                             ScannerId::slave0,
                                                             ScannerId::slave1,
                                                             ScannerId::slave2 };

static const std::map<ScannerId, std::string> scanner_id_to_string{ { ScannerId::master, "Master" },
                                                                    { ScannerId::slave0, "Slave0" },
                                                                    { ScannerId::slave1, "Slave1" },
                                                                    { ScannerId::slave2, "Slave2" } };

}  // namespace configuration
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_IDS_H
