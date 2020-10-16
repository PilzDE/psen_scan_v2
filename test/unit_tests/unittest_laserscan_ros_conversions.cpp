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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/scanner.h"
#include "psen_scan_v2/laserscan_ros_conversions.h"

using namespace psen_scan_v2;

MATCHER_P(IsReversed, data_vec, "")
{
  auto data_vec_copy = data_vec;
  std::reverse(data_vec_copy.begin(), data_vec_copy.end());
  return arg == data_vec_copy;
}

const double EPSILON{1.0e-8};

namespace psen_scan_v2_test
{
TEST(LaserScanROSConversionsTest, testToLaserScanMsg)
{
  const std::string frame_id{ "frame_id" };
  constexpr double x_axis_rotation{ 0 };

  const TenthOfDegree angle_min_raw{ 0 };
  const TenthOfDegree angle_max_raw{ 20 };
  const TenthOfDegree angle_increment{ 1 };
  LaserScan laserscan{ angle_increment, angle_min_raw, angle_max_raw };
  const MeasurementData measures{ 1, 2, 3 };
  laserscan.setMeasurements(measures);

  ros::Time now = ros::Time::now();
  sensor_msgs::LaserScan laserscan_msg = toLaserScanMsg(laserscan, frame_id, x_axis_rotation, now);

  EXPECT_EQ(laserscan_msg.header.seq, 0u);
  EXPECT_EQ(laserscan_msg.header.stamp, now);
  EXPECT_EQ(laserscan_msg.header.frame_id, frame_id);

  EXPECT_NEAR(laserscan_msg.angle_min, x_axis_rotation - angle_max_raw.toRad(), EPSILON);
  EXPECT_NEAR(laserscan_msg.angle_max, x_axis_rotation - angle_min_raw.toRad(), EPSILON);

  EXPECT_NEAR(laserscan_msg.angle_increment, angle_increment.toRad(), EPSILON);
  EXPECT_NEAR(laserscan_msg.time_increment, TIME_PER_SCAN_IN_S / measures.size(), EPSILON);

  EXPECT_NEAR(laserscan_msg.scan_time, TIME_PER_SCAN_IN_S, EPSILON);

  EXPECT_EQ(laserscan_msg.range_min, RANGE_MIN_IN_M);
  EXPECT_EQ(laserscan_msg.range_max, RANGE_MAX_IN_M);

  ASSERT_EQ(laserscan_msg.ranges.size(), measures.size());

  // Check that the ranges in the ROS msg is the same order as the laserscan and given in meters
  auto reverse_it_measures = measures.cbegin();
  auto iter_ranges = laserscan_msg.ranges.cbegin();
  while (reverse_it_measures != measures.end() && iter_ranges != laserscan_msg.ranges.end())
  {
    EXPECT_NEAR(*iter_ranges, *reverse_it_measures, EPSILON);
    iter_ranges++;
    reverse_it_measures++;
  }

  EXPECT_TRUE(laserscan_msg.intensities.empty());
}
}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
