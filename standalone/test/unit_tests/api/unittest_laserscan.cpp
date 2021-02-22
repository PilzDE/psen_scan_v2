// Copyright (c) 2019-2021 Pilz GmbH & Co. KG
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

#include <memory>
#include <stdexcept>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/laserscan.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static const util::TenthOfDegree DEFAULT_RESOLUTION{ 1 };
static const util::TenthOfDegree DEFAULT_START_ANGLE{ 1 };
static const util::TenthOfDegree DEFAULT_END_ANGLE{ 2 };

TEST(LaserScanTests, testInvalidZeroResolution)
{
  EXPECT_THROW(LaserScan laser_scan(util::TenthOfDegree(0), DEFAULT_START_ANGLE, DEFAULT_END_ANGLE),
               std::invalid_argument);
}

TEST(LaserScanTests, testOutOfRangeResolution)
{
  EXPECT_THROW(LaserScan laser_scan(util::TenthOfDegree(10000), DEFAULT_START_ANGLE, DEFAULT_END_ANGLE),
               std::invalid_argument);
}

TEST(LaserScanTests, testInvalidStartEndAngle)
{
  EXPECT_THROW(LaserScan laser_scan(DEFAULT_RESOLUTION, DEFAULT_END_ANGLE, DEFAULT_START_ANGLE), std::invalid_argument);
}

TEST(LaserScanTest, testGetScanResolution)
{
  const util::TenthOfDegree expected_resolution{ DEFAULT_RESOLUTION };
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(expected_resolution, DEFAULT_START_ANGLE, DEFAULT_END_ANGLE)););

  EXPECT_EQ(expected_resolution, laser_scan->getScanResolution());
}

TEST(LaserScanTest, testGetMinScanAngle)
{
  const auto expected_min_scan_angle{ DEFAULT_START_ANGLE };
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(DEFAULT_RESOLUTION, expected_min_scan_angle, DEFAULT_END_ANGLE)););

  EXPECT_EQ(expected_min_scan_angle, laser_scan->getMinScanAngle());
}

TEST(LaserScanTest, testGetMaxScanAngle)
{
  const auto expected_max_scan_angle{ DEFAULT_END_ANGLE };
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(DEFAULT_RESOLUTION, DEFAULT_START_ANGLE, expected_max_scan_angle)););

  EXPECT_EQ(expected_max_scan_angle, laser_scan->getMaxScanAngle());
}

TEST(LaserScanTest, testMinEqualsMax)
{
  ASSERT_NO_THROW(LaserScan(DEFAULT_RESOLUTION, util::TenthOfDegree(1000u), util::TenthOfDegree(1000u)););
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
