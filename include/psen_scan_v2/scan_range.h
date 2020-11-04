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

#ifndef PSEN_SCAN_V2_SCAN_RANGE_H
#define PSEN_SCAN_V2_SCAN_RANGE_H

#include <stdexcept>

#include "psen_scan_v2/tenth_of_degree.h"

namespace psen_scan_v2
{
/**
 * @brief Higher level data type storing the range in which the scanner takes measurements.
 */
template <int16_t min_allowed_angle, int16_t max_allowed_angle>
class ScanRange
{
  static_assert(min_allowed_angle < max_allowed_angle, "MIN-angle limit smaller than MAX-angle limit.");

public:
  ScanRange() = default;

  /**
   * @brief Constructor.
   *
   * @param start_angle Start angle of measurement (scanner-zero = zero on the left).
   * @param end_angle End angle of measurement.
   */
  constexpr ScanRange(const TenthOfDegree& start_angle, const TenthOfDegree& end_angle);

  const TenthOfDegree& getStart() const;
  const TenthOfDegree& getEnd() const;

private:
  TenthOfDegree start_angle_{ 0 };
  TenthOfDegree end_angle_{ 0 };

  const TenthOfDegree MIN_ANGLE{ min_allowed_angle };
  const TenthOfDegree MAX_ANGLE{ max_allowed_angle };
};

template <int16_t min_angle, int16_t max_angle>
constexpr ScanRange<min_angle, max_angle>::ScanRange(const TenthOfDegree& start_angle, const TenthOfDegree& end_angle)
  : start_angle_(start_angle), end_angle_(end_angle)
{
  if (start_angle < MIN_ANGLE || start_angle > MAX_ANGLE)
  {
    throw std::out_of_range("Start angle out of range");
  }

  if (end_angle < MIN_ANGLE || end_angle > MAX_ANGLE)
  {
    throw std::out_of_range("End angle out of range");
  }

  if (start_angle > end_angle)
  {
    throw std::invalid_argument("Start angle must be smaller or equal to end angle");
  }
}

template <int16_t min_angle, int16_t max_angle>
const TenthOfDegree& ScanRange<min_angle, max_angle>::getStart() const
{
  return start_angle_;
}

template <int16_t min_angle, int16_t max_angle>
const TenthOfDegree& ScanRange<min_angle, max_angle>::getEnd() const
{
  return end_angle_;
}

using DefaultScanRange = ScanRange<0, 2750>;

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCAN_RANGE_H
