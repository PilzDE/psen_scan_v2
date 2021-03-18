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

#include <cstdint>
#include <stdexcept>
#include <limits>

#include <boost/math/constants/constants.hpp>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"

namespace psen_scan_v2_standalone_test
{
TEST(TenthDegreeConversionTests, radToTenthDegreeShouldReturnExpectedPositiveValue)
{
  const int16_t expected_tenth_degree{ 11 };
  const double angle_in_rad{ (static_cast<double>(expected_tenth_degree) / 1800.) * boost::math::double_constants::pi };
  EXPECT_EQ(expected_tenth_degree, psen_scan_v2_standalone::data_conversion_layer::radToTenthDegree(angle_in_rad));
}

TEST(TenthDegreeConversionTests, radToTenthDegreeShouldReturnExpectedNegativeValue)
{
  const int16_t expected_tenth_degree{ -11 };
  const double angle_in_rad{ (static_cast<double>(expected_tenth_degree) / 1800.) * boost::math::double_constants::pi };
  EXPECT_EQ(expected_tenth_degree, psen_scan_v2_standalone::data_conversion_layer::radToTenthDegree(angle_in_rad));
}

TEST(TenthDegreeConversionTests, radToTenthDegreeShouldThrowWhenAngleOutOfRange)
{
  const double max_exceeding_angle_in_rad{ static_cast<double>(std::numeric_limits<int16_t>::max()) + .1 };
  EXPECT_THROW(psen_scan_v2_standalone::data_conversion_layer::radToTenthDegree(max_exceeding_angle_in_rad),
               std::invalid_argument);

  const double min_exceeding_angle_in_rad{ static_cast<double>(std::numeric_limits<int16_t>::min()) - .1 };
  EXPECT_THROW(psen_scan_v2_standalone::data_conversion_layer::radToTenthDegree(min_exceeding_angle_in_rad),
               std::invalid_argument);
}

TEST(TenthDegreeConversionTests, tenthDegreeToRadShouldReturnExpectedValue)
{
  const double expected_radian{ 0.01 * boost::math::double_constants::pi };
  const int16_t angle_in_tenth_degree{ 18 };
  EXPECT_DOUBLE_EQ(expected_radian,
                   psen_scan_v2_standalone::data_conversion_layer::tenthDegreeToRad(angle_in_tenth_degree));
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
