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

#ifndef PSEN_SCAN_V2_FORMAT_RANGE_H
#define PSEN_SCAN_V2_FORMAT_RANGE_H

#include <algorithm>

#include <sstream>
#include <string>

#include <fmt/format.h>

template <typename T>
std::string formatRange(const T& range)
{
  std::stringstream strstr{ "{" };
  std::for_each(
      range.begin(), std::prev(range.end()), [&strstr](const auto& el) { strstr << fmt::format("{}", el) << ", "; });
  strstr << fmt::format("{}", *std::prev(range.end())) << "}";
  return strstr.str();
}

#endif  // PSEN_SCAN_V2_FORMAT_RANGE_H
