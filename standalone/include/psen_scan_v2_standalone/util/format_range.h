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

#ifndef PSEN_SCAN_V2_STANDALONE_FORMAT_RANGE_H
#define PSEN_SCAN_V2_STANDALONE_FORMAT_RANGE_H

#include <algorithm>
#include <iterator>

#include <sstream>
#include <string>

#include <fmt/format.h>

namespace psen_scan_v2_standalone
{
namespace util
{
/**
 * The resulting string will look like the following:
 * \verbatim "{Item1, Item2, [...], ItemLast}" \endverbatim
 * Examples:
 * \verbatim "{1, 2, 3}" \endverbatim
 * \verbatim "{}" (Empty Range) \endverbatim
 */
template <typename T>
std::string formatRange(const T& range)
{
  std::stringstream strstr;
  strstr << "{";
  for (auto it = range.begin(); std::next(it) < range.end(); ++it)
  {
    strstr << fmt::format("{}, ", *it);
  }
  if (range.begin() < range.end())
  {
    strstr << fmt::format("{}", *std::prev(range.end()));
  }
  strstr << "}";
  return strstr.str();
}

}  // namespace util
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_FORMAT_RANGE_H
