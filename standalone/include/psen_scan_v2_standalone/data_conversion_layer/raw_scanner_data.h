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
#ifndef PSEN_SCAN_V2_STANDALONE_RAW_SCANNER_DATA_H
#define PSEN_SCAN_V2_STANDALONE_RAW_SCANNER_DATA_H

#include <memory>
#include <vector>

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
using RawData = std::vector<char>;
static constexpr std::size_t MAX_UDP_PAKET_SIZE{ 65507 };
using RawDataPtr = std::shared_ptr<RawData>;
using RawDataConstPtr = std::shared_ptr<const RawData>;
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_RAW_SCANNER_DATA_H
