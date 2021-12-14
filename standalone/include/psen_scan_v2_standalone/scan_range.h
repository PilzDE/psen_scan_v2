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

#ifndef PSEN_SCAN_V2_STANDALONE_SCAN_RANGE_H
#define PSEN_SCAN_V2_STANDALONE_SCAN_RANGE_H

#include <stdexcept>

#include "psen_scan_v2_standalone/util/tenth_of_degree.h"

namespace psen_scan_v2_standalone
{
/**
 * @brief Higher level data type storing the range in which the scanner takes measurements.
 *
 * A ScanRange is given by a start- and an end-angle, which are represented in TenthOfDegree.
 */
template <int16_t min_allowed_angle, int16_t max_allowed_angle>
class ScanRangeTemplated
{
  static_assert(min_allowed_angle < max_allowed_angle, "MIN-angle limit smaller than MAX-angle limit.");

public:
  /**
   * @brief Constructor.
   *
   * @param start_angle Start angle of measurement (scanner-zero = zero on the left).
   * @param end_angle End angle of measurement. Closed interval. (End angle included.)
   */
  constexpr ScanRangeTemplated(const util::TenthOfDegree& start_angle, const util::TenthOfDegree& end_angle);

  /*! deprecated: use const util::TenthOfDegree& start() const instead */
  [[deprecated("use const util::TenthOfDegree& start() const instead")]] const util::TenthOfDegree& getStart() const;
  const util::TenthOfDegree& start() const;

  /*! deprecated: use const util::TenthOfDegree& end() const instead */
  [[deprecated("use const util::TenthOfDegree& end() const const instead")]] const util::TenthOfDegree& getEnd() const;
  const util::TenthOfDegree& end() const;

public:
  static constexpr ScanRangeTemplated<min_allowed_angle, max_allowed_angle> createInvalidScanRange();

private:
  ScanRangeTemplated() = default;

private:
  util::TenthOfDegree start_angle_{ 0 };
  util::TenthOfDegree end_angle_{ 0 };
};

template <int16_t min_angle_raw, int16_t max_angle_raw>
constexpr ScanRangeTemplated<min_angle_raw, max_angle_raw>::ScanRangeTemplated(const util::TenthOfDegree& start_angle,
                                                                               const util::TenthOfDegree& end_angle)
  : start_angle_(start_angle), end_angle_(end_angle)
{
  const util::TenthOfDegree min_angle{ min_angle_raw };
  const util::TenthOfDegree max_angle{ max_angle_raw };

  if (start_angle < min_angle || start_angle > max_angle)
  {
    throw std::out_of_range("Start angle out of range");
  }

  if (end_angle < min_angle || end_angle > max_angle)
  {
    throw std::out_of_range("End angle out of range");
  }

  if (start_angle >= end_angle)
  {
    throw std::invalid_argument("Start angle must be smaller than end angle");
  }
}

template <int16_t min_angle, int16_t max_angle>
constexpr ScanRangeTemplated<min_angle, max_angle> ScanRangeTemplated<min_angle, max_angle>::createInvalidScanRange()
{
  return ScanRangeTemplated<min_angle, max_angle>();
}

template <int16_t min_angle, int16_t max_angle>
const util::TenthOfDegree& ScanRangeTemplated<min_angle, max_angle>::start() const
{
  return start_angle_;
}

template <int16_t min_angle, int16_t max_angle>
const util::TenthOfDegree& ScanRangeTemplated<min_angle, max_angle>::end() const
{
  return end_angle_;
}

template <int16_t min_angle, int16_t max_angle>
const util::TenthOfDegree& ScanRangeTemplated<min_angle, max_angle>::getStart() const
{
  return start_angle_;
}

template <int16_t min_angle, int16_t max_angle>
const util::TenthOfDegree& ScanRangeTemplated<min_angle, max_angle>::getEnd() const
{
  return end_angle_;
}

using ScanRange = ScanRangeTemplated<1, 2749>;

}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCAN_RANGE_H
