// Copyright (c) 2019-2022 Pilz GmbH & Co. KG
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

#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"
#include "psen_scan_v2_standalone/configuration/scanner_ids.h"

#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data_helper.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static const util::TenthOfDegree DEFAULT_RESOLUTION{ 1 };
static const util::TenthOfDegree DEFAULT_MIN_SCAN_ANGLE{ 1 };
static const util::TenthOfDegree DEFAULT_MAX_SCAN_ANGLE{ 2 };
static const uint32_t DEFAULT_SCAN_COUNTER{ 1 };
static const uint8_t DEFAULT_ACTIVE_ZONESET{ 2 };
static const int64_t DEFAULT_TIMESTAMP{ 1 };
static const configuration::ScannerId DEFAULT_SCANNER_ID{ configuration::ScannerId::master };

class LaserScanBuilder
{
public:
  LaserScan build()
  {
    return LaserScan(
      resolution_, min_scan_angle_, max_scan_angle_, scan_counter_, active_zoneset_, timestamp_, scanner_id_);
  }

  LaserScanBuilder& resolution(const util::TenthOfDegree& resolution)
  {
    resolution_ = resolution;
    return *this;
  }

  LaserScanBuilder& minScanAngle(const util::TenthOfDegree& min_scan_angle)
  {
    min_scan_angle_ = min_scan_angle;
    return *this;
  }

  LaserScanBuilder& maxScanAngle(const util::TenthOfDegree& max_scan_angle)
  {
    max_scan_angle_ = max_scan_angle;
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

  LaserScanBuilder& scannerId(const configuration::ScannerId scanner_id)
  {
    scanner_id_ = scanner_id;
    return *this;
  }

private:
  util::TenthOfDegree resolution_{ DEFAULT_RESOLUTION };
  util::TenthOfDegree min_scan_angle_{ DEFAULT_MIN_SCAN_ANGLE };
  util::TenthOfDegree max_scan_angle_{ DEFAULT_MAX_SCAN_ANGLE };
  uint32_t scan_counter_{ DEFAULT_SCAN_COUNTER };
  uint8_t active_zoneset_{ DEFAULT_ACTIVE_ZONESET };
  int64_t timestamp_{ DEFAULT_TIMESTAMP };
  configuration::ScannerId scanner_id_{ DEFAULT_SCANNER_ID };
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
  laser_scan_builder.minScanAngle(DEFAULT_MAX_SCAN_ANGLE).maxScanAngle(DEFAULT_MIN_SCAN_ANGLE);
  EXPECT_THROW(laser_scan_builder.build(), std::invalid_argument);
}

TEST(LaserScanTest, testGetScanResolution)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_RESOLUTION, laser_scan->scanResolution());
}

TEST(LaserScanTest, testGetMinScanAngle)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_MIN_SCAN_ANGLE, laser_scan->minScanAngle());
}

TEST(LaserScanTest, testGetMaxScanAngle)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_MAX_SCAN_ANGLE, laser_scan->maxScanAngle());
}

TEST(LaserScanTest, testMinEqualsMax)
{
  LaserScanBuilder laser_scan_builder;
  laser_scan_builder.minScanAngle(util::TenthOfDegree(1000u)).maxScanAngle(util::TenthOfDegree(1000u));
  ASSERT_NO_THROW(laser_scan_builder.build(););
}

TEST(LaserScanTest, testGetScanCounter)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_SCAN_COUNTER, laser_scan->scanCounter());
}

TEST(LaserScanTest, testGetTimestamp)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_TIMESTAMP, laser_scan->timestamp());
}

TEST(LaserScanTest, testGetActiveZoneset)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););
  EXPECT_EQ(DEFAULT_ACTIVE_ZONESET, laser_scan->activeZoneset());
}

TEST(LaserScanTest, testSetAndGetIOStates)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););

  IOState io_state{ IOState(createPinData(), 42 /*timestamp*/) };
  laser_scan->ioStates({ io_state });
  EXPECT_EQ(laser_scan->ioStates()[0].input(), io_state.input());
  EXPECT_EQ(laser_scan->ioStates()[0].output(), io_state.output());
  EXPECT_EQ(laser_scan->ioStates()[0].timestamp(), 42);
}

TEST(LaserScanTest, testPrintMessageSuccess)
{
  LaserScanBuilder laser_scan_builder;
  std::unique_ptr<LaserScan> laser_scan;
  ASSERT_NO_THROW(laser_scan.reset(new LaserScan(laser_scan_builder.build())););

  laser_scan->measurements({ 45.0, 44.0, 43.0, 42.0 });
  laser_scan->ioStates({ IOState(createPinData(), 41 /*timestamp*/) });

// For compatibility with different ubuntu versions (resp. fmt), we need to take account of changes in
// the default formatting of floating point numbers
#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
  EXPECT_EQ(
      fmt::format("{}", *laser_scan),
      "LaserScan(timestamp = 1 nsec, scanCounter = 1, minScanAngle = 0.1 deg, maxScanAngle = 0.2 deg, resolution "
      "= 0.1 deg, active_zoneset = 2, measurements = {45.0, 44.0, 43.0, 42.0}, intensities = {}, io_states = "
      "{IOState(timestamp = 41 nsec, io::PinData(input = {01001101, 00000000, 00000000, 00000000, 10011010, 00000000, "
      "00000000, 00000000}, output = {01010101, 00000000, 00000000, 00000000}))})");
#else
  EXPECT_EQ(
      fmt::format("{}", *laser_scan),
      "LaserScan(timestamp = 1 nsec, scanCounter = 1, minScanAngle = 0.1 deg, maxScanAngle = 0.2 deg, resolution "
      "= 0.1 deg, active_zoneset = 2, measurements = {45, 44, 43, 42}, intensities = {}, io_states = "
      "{IOState(timestamp = 41 nsec, io::PinData(input = {01001101, 00000000, 00000000, 00000000, 10011010, 00000000, "
      "00000000, 00000000}, output = {01010101, 00000000, 00000000, 00000000}))})");
#endif
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
