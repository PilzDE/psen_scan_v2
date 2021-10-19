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

#ifndef PSEN_SCAN_V2_ZONESET_H
#define PSEN_SCAN_V2_ZONESET_H

#include <vector>

#include <fmt/format.h>

#include <boost/optional.hpp>

namespace psen_scan_v2_standalone
{
namespace configuration
{
class ZoneSetSpeedRangeException : public std::runtime_error
{
public:
  ZoneSetSpeedRangeException(const std::string& msg) : std::runtime_error(msg)
  {
  }
};
class ZoneSetSpeedRange
{
public:
  ZoneSetSpeedRange(short min, short max) : min_(min), max_(max)
  {
    if (min > max)
    {
      throw ZoneSetSpeedRangeException(fmt::format("Invalid speedrange min: {} > max: {}", min, max));
    }
  };
  short min_{ 0 };
  short max_{ 0 };

  bool operator==(const ZoneSetSpeedRange& rhs) const
  {
    return (min_ == rhs.min_) && (max_ == rhs.max_);
  }
  bool operator!=(const ZoneSetSpeedRange& rhs) const
  {
    return !operator==(rhs);
  }
};

class ZoneSet
{
public:
  // TODO replace with proper polar coordinates
  std::vector<unsigned long> safety1_;
  std::vector<unsigned long> safety2_;
  std::vector<unsigned long> safety3_;
  std::vector<unsigned long> warn1_;
  std::vector<unsigned long> warn2_;
  std::vector<unsigned long> muting1_;
  std::vector<unsigned long> muting2_;

  boost::optional<ZoneSetSpeedRange> speed_range_;
};

}  // namespace configuration
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_ZONESET_H
