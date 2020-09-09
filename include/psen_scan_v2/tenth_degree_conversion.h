#ifndef PSEN_SCAN_V2_TENTH_DEGREE_CONVERSION_H
#define PSEN_SCAN_V2_TENTH_DEGREE_CONVERSION_H

#include <cstdint>
#include <stdexcept>
#include <limits>
#include <sstream>

#include <boost/math/constants/constants.hpp>

namespace psen_scan_v2
{
static constexpr double radianToDegree(const double& angle_in_rad)
{
  return angle_in_rad * 180. / boost::math::double_constants::pi;
}

static uint16_t radToTenthDegree(const double& angle_in_rad)
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

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_TENTH_DEGREE_CONVERSION_H
