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

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/laserscan.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static const util::TenthOfDegree DEFAULT_RESOLUTION{ 1 };
static const util::TenthOfDegree DEFAULT_START_ANGLE{ 1 };
static const util::TenthOfDegree DEFAULT_END_ANGLE{ 2 };
static const int64_t DEFAULT_TIMESTAMP{ 1 };
static const uint32_t DEFAULT_SCAN_COUNTER{ 1 };

class LaserScanBuilder
{
public:
  LaserScan build()
  {
    return LaserScan(resolution_, start_angle_, end_angle_, scan_counter_, timestamp_);
  }

  LaserScanBuilder& resolution(const util::TenthOfDegree& resolution)
  {
    resolution_ = resolution;
    return *this;
  }

  LaserScanBuilder& startAngle(const util::TenthOfDegree& start_angle)
  {
    start_angle_ = start_angle;
    return *this;
  }

  LaserScanBuilder& endAngle(const util::TenthOfDegree& end_angle)
  {
    end_angle_ = end_angle;
    return *this;
  }

  LaserScanBuilder& scanCounter(const uint32_t scan_counter)
  {
    scan_counter_ = scan_counter;
    return *this;
  }

  LaserScanBuilder& timestamp(const int64_t timestamp)
  {
    timestamp_ = timestamp;
    return *this;
  }

private:
  util::TenthOfDegree resolution_{ DEFAULT_RESOLUTION };
  util::TenthOfDegree start_angle_{ DEFAULT_START_ANGLE };
  util::TenthOfDegree end_angle_{ DEFAULT_END_ANGLE };
  uint32_t scan_counter_{ DEFAULT_SCAN_COUNTER };
  int64_t timestamp_{ DEFAULT_TIMESTAMP };
};

TEST(LaserScanTests, testInvalidZeroResolution)
{
  LaserScanBuilder laser_scan_builder;
  laser_scan_builder.resolution(util::TenthOfDegree(0));
  EXPECT_THROW(laser_scan_builder.build(), std::invalid_argument);
}

TEST(LaserScanTests, testOutOfRangeResolution)
{
  LaserScanBuilder laser_scan_builder;
  laser_scan_builder.resolution(util::TenthOfDegree(10000));
  EXPECT_THROW(laser_scan_builder.build(), std::invalid_argument);
}

TEST(LaserScanTests, testInvalidStartEndAngle)
{
  LaserScanBuilder laser_scan_builder;
  laser_scan_builder.startAngle(DEFAULT_END_ANGLE).endAngle(DEFAULT_START_ANGLE);
  EXPECT_THROW(laser_scan_builder.build(), std::invalid_argument);
}

TEST(LaserScanTest, testGetScanResolution)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_RESOLUTION, laser_scan->getScanResolution());
}

TEST(LaserScanTest, testGetMinScanAngle)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_START_ANGLE, laser_scan->getMinScanAngle());
}

TEST(LaserScanTest, testGetMaxScanAngle)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_END_ANGLE, laser_scan->getMaxScanAngle());
}

TEST(LaserScanTest, testMinEqualsMax)
{
  LaserScanBuilder laser_scan_builder;
  laser_scan_builder.startAngle(util::TenthOfDegree(1000u)).endAngle(util::TenthOfDegree(1000u));
  ASSERT_NO_THROW(laser_scan_builder.build(););
}

TEST(LaserScanTest, testGetScanCounter)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_SCAN_COUNTER, laser_scan->getScanCounter());
}

TEST(LaserScanTest, testGetTimestamp)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_TIMESTAMP, laser_scan->getTimestamp());
}

TEST(LaserScanTest, testPrintMessageSuccess)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););

  laser_scan->setMeasurements({ 45.0, 44.0, 43.0, 42.0 });

// For compatibility with different ubuntu versions (resp. fmt), we need to take account of changes in
// the default formatting of floating point numbers
#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
  EXPECT_EQ(fmt::format("{}", *laser_scan),
            "LaserScan(timestamp = 1 nsec, scanCounter = 1, minScanAngle = 0.1 deg, maxScanAngle = 0.2 deg, resolution "
            "= 0.1 deg, measurements = {45.0, 44.0, 43.0, 42.0}, intensities = {})");
#else
  EXPECT_EQ(fmt::format("{}", *laser_scan),
            "LaserScan(timestamp = 1 nsec, scanCounter = 1, minScanAngle = 0.1 deg, maxScanAngle = 0.2 deg, resolution "
            "= 0.1 deg, measurements = {45, 44, 43, 42}, intensities = {})");
#endif
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
