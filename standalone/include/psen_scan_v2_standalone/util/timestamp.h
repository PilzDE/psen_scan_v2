// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_TIMESTAMP_H
#define PSEN_SCAN_V2_STANDALONE_TIMESTAMP_H

#include <chrono>

namespace psen_scan_v2_standalone
{
namespace util
{
inline static auto getCurrentTime()
{
  return std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now())
      .time_since_epoch()
      .count();
}
}  // namespace util
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_TIMESTAMP_H
