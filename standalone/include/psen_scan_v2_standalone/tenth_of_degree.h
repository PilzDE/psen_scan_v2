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

#ifndef PSEN_SCAN_V2_STANDALONE_TENTH_OF_DEGREE_H
#define PSEN_SCAN_V2_STANDALONE_TENTH_OF_DEGREE_H

#include "psen_scan_v2_standalone/angle_conversions.h"

namespace psen_scan_v2_standalone
{
/**
 * @brief Helper class to simplify/improve the handling of angles.
 *
 * There are different representations for angles on the scanner side and the api.
 * - The scanner uses tenth of degree
 * - The driver uses radian to be conform to ROS.
 *
 * This class helps to convert those angles back and forth and to keep consistency within the driver.
 */
class TenthOfDegree
{
public:
  static TenthOfDegree fromRad(const double& angle_in_rad)
  {
    return TenthOfDegree(radToTenthDegree(angle_in_rad));
  }

public:
  explicit constexpr TenthOfDegree(const int16_t& tenth_of_degree) : tenth_of_degree_(tenth_of_degree)
  {
  }

  constexpr int16_t value() const
  {
    return tenth_of_degree_;
  }

  constexpr double toRad() const
  {
    return tenthDegreeToRad(tenth_of_degree_);
  }

  constexpr TenthOfDegree& operator*(const TenthOfDegree& rhs)
  {
    tenth_of_degree_ = value() * rhs.value();
    return *this;
  }

  constexpr TenthOfDegree& operator*(const int& rhs)
  {
    tenth_of_degree_ = value() * rhs;
    return *this;
  }

  constexpr TenthOfDegree& operator+(const TenthOfDegree& rhs)
  {
    tenth_of_degree_ = value() + rhs.value();
    return *this;
  }

  constexpr bool operator==(const TenthOfDegree& rhs) const
  {
    return value() == rhs.value();
  }

  constexpr bool operator>=(const TenthOfDegree& rhs) const
  {
    return value() >= rhs.value();
  }

  constexpr bool operator<=(const TenthOfDegree& rhs) const
  {
    return value() <= rhs.value();
  }

  constexpr bool operator>(const TenthOfDegree& rhs) const
  {
    return value() > rhs.value();
  }

  constexpr bool operator<(const TenthOfDegree& rhs) const
  {
    return value() < rhs.value();
  }

private:
  int16_t tenth_of_degree_{ 0 };
};
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_TENTH_OF_DEGREE_H
