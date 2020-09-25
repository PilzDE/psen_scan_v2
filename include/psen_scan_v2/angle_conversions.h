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

#ifndef PSEN_SCAN_V2_ANGLE_CONVERSIONS_H
#define PSEN_SCAN_V2_ANGLE_CONVERSIONS_H

#include <boost/math/constants/constants.hpp>

namespace psen_scan_v2
{
inline static constexpr double radianToDegree(const double& angle_in_rad)
{
  return angle_in_rad * 180. / boost::math::double_constants::pi;
}

inline static constexpr double degreeToRadian(const double& angle_in_degree)
{
  return (angle_in_degree / 180.) * boost::math::double_constants::pi;
}

inline static uint16_t radToTenthDegree(const double& angle_in_rad)
{
  const double tenth_degree_rounded{ std::round(10. * radianToDegree(angle_in_rad)) };
  if (tenth_degree_rounded < std::numeric_limits<uint16_t>::min() ||
      tenth_degree_rounded > std::numeric_limits<uint16_t>::max())
  {
    std::stringstream exception_msg;
    exception_msg << "Angle " << tenth_degree_rounded << " (tenth of degree) is out of range.";
    throw std::invalid_argument(exception_msg.str());
  }
  return static_cast<uint16_t>(tenth_degree_rounded);
}

inline static double tenthDegreeToRad(const uint16_t& angle_in_tenth_degree)
{
  return degreeToRadian(static_cast<double>(angle_in_tenth_degree) / 10.);
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ANGLE_CONVERSIONS_H
