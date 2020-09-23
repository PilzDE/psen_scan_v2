// Copyright (c) 2019-2020 Pilz GmbH & Co. KG
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

#include <stdexcept>

#include <gtest/gtest.h>

#include "psen_scan_v2/laserscan.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
static constexpr double DEFAULT_RESOLUTION{ 0.0174533 };
static constexpr double DEFAULT_START_ANGLE{ 0.0174533 };
static constexpr double DEFAULT_END_ANGLE{ 0.0349066 };

TEST(LaserScanTests, testInvalidZeroResolution)
{
  EXPECT_THROW(LaserScan laser_scan(0., DEFAULT_START_ANGLE, DEFAULT_END_ANGLE), std::invalid_argument);
}

TEST(LaserScanTests, testOutOfRangeResolution)
{
  EXPECT_THROW(LaserScan laser_scan(1000. * DEFAULT_RESOLUTION, DEFAULT_START_ANGLE, DEFAULT_END_ANGLE),
               std::invalid_argument);
}

TEST(LaserScanTests, testInvalidStartEndAngle)
{
  EXPECT_THROW(LaserScan laser_scan(DEFAULT_RESOLUTION, DEFAULT_END_ANGLE, DEFAULT_START_ANGLE), std::invalid_argument);
}

}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
