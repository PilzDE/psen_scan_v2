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

#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/configuration/default_parameters.h"

#include "psen_scan_v2/laserscan_ros_conversions.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_standalone::configuration;

const double EPSILON{ 1.0e-8 };

namespace psen_scan_v2_test
{
static LaserScan createScan(int64_t stamp = 1)
{
  const util::TenthOfDegree angle_min_raw{ 0 };
  const util::TenthOfDegree angle_max_raw{ 20 };
  const util::TenthOfDegree angle_increment{ 1 };
  const uint32_t scan_counter{ 1 };
  const uint8_t active_zoneset{ 0 };
  const int64_t timestamp{ stamp };

  LaserScan laserscan(angle_increment, angle_min_raw, angle_max_raw, scan_counter, active_zoneset, timestamp);
  const LaserScan::MeasurementData measurements{ 1., 2., 3. };
  laserscan.measurements(measurements);

  const LaserScan::IntensityData intensities{ 707., 304., 0. };
  laserscan.intensities(intensities);

  return laserscan;
}

TEST(LaserScanROSConversionsTest, laserSensorMsgShouldContainCorrectHeaderAfterConversion)
{
  const std::string prefix{ "prefix" };
  const LaserScan laserscan{ createScan() };
  const sensor_msgs::LaserScan laserscan_msg = toLaserScanMsg(laserscan, prefix, 0);

  EXPECT_EQ(laserscan_msg.header.seq, 0u);
  EXPECT_EQ(static_cast<int64_t>(laserscan_msg.header.stamp.toNSec()), laserscan.timestamp());
  EXPECT_EQ(laserscan_msg.header.frame_id, prefix);
}

TEST(LaserScanROSConversionsTest, laserSensorMsgShouldContainCorrectScanResolutionAfterConversion)
{
  const LaserScan laserscan{ createScan() };
  const sensor_msgs::LaserScan laserscan_msg = toLaserScanMsg(laserscan, "", 0);

  EXPECT_NEAR(laserscan_msg.angle_increment, laserscan.scanResolution().toRad(), EPSILON)
      << "Resolution incorrect in sensor_msgs::LaserScan";
}

TEST(LaserScanROSConversionsTest, laserSensorMsgShouldContainCorrectMinMaxScanAngleAfterConversion)
{
  const LaserScan laserscan{ createScan() };
  constexpr double x_axis_rotation{ 0 };
  const sensor_msgs::LaserScan laserscan_msg = toLaserScanMsg(laserscan, "", x_axis_rotation);

  EXPECT_NEAR(laserscan_msg.angle_min, laserscan.minScanAngle().toRad() - x_axis_rotation, EPSILON);
  EXPECT_NEAR(laserscan_msg.angle_max, laserscan.maxScanAngle().toRad() - x_axis_rotation, EPSILON);
}

TEST(LaserScanROSConversionsTest, laserSensorMsgShouldContainCorrectTimePerRadAfterConversion)
{
  const LaserScan laserscan{ createScan() };
  const sensor_msgs::LaserScan laserscan_msg = toLaserScanMsg(laserscan, "", 0);

  const double time_per_rad = configuration::TIME_PER_SCAN_IN_S / (2 * M_PI);  // angle speed
  EXPECT_NEAR(laserscan_msg.time_increment, time_per_rad * laserscan.scanResolution().toRad(), EPSILON);
}

TEST(LaserScanROSConversionsTest, laserSensorMsgShouldContainCorrectMinMaxRangeAfterConversion)
{
  const sensor_msgs::LaserScan laserscan_msg = toLaserScanMsg(createScan(), "", 0);

  EXPECT_NEAR(laserscan_msg.range_min, configuration::RANGE_MIN_IN_M, EPSILON);
  EXPECT_NEAR(laserscan_msg.range_max, configuration::RANGE_MAX_IN_M, EPSILON);
}

TEST(LaserScanROSConversionsTest, laserSensorMsgShouldContainCorrectScanTimeAfterConversion)
{
  const sensor_msgs::LaserScan laserscan_msg = toLaserScanMsg(createScan(), "", 0);

  EXPECT_NEAR(laserscan_msg.scan_time, configuration::TIME_PER_SCAN_IN_S, EPSILON);
}

TEST(LaserScanROSConversionsTest, laserSensorMsgShouldContainCorrectRangesAfterConversion)
{
  const LaserScan laserscan{ createScan() };
  const sensor_msgs::LaserScan laserscan_msg = toLaserScanMsg(laserscan, "", 0);

  ASSERT_EQ(laserscan_msg.ranges.size(), laserscan.measurements().size());
  // Check that the ranges in the ROS msg is the same order as the laserscan and given in meters
  for (size_t i = 0; i < laserscan_msg.ranges.size(); ++i)
  {
    EXPECT_NEAR(laserscan_msg.ranges.at(i), laserscan.measurements().at(i), EPSILON);
  }
}

TEST(LaserScanROSConversionsTest, laserSensorMsgShouldContainCorrectIntensitiesAfterConversion)
{
  const LaserScan laserscan{ createScan() };
  const sensor_msgs::LaserScan laserscan_msg = toLaserScanMsg(laserscan, "", 0);

  ASSERT_EQ(laserscan_msg.intensities.size(), laserscan.intensities().size());
  for (size_t i = 0; i < laserscan_msg.intensities.size(); ++i)
  {
    EXPECT_NEAR(laserscan_msg.intensities.at(i), laserscan.intensities().at(i), EPSILON);
  }
}

TEST(LaserScanROSConversionsTest, shouldThrowIfLaserScanHasNegativeTimestamp)
{
  const LaserScan laserscan{ createScan(-1) };
  EXPECT_THROW(toLaserScanMsg(laserscan, "", 0), std::invalid_argument);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
