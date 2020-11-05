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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <functional>

#include <algorithm>
#include <future>
#include <iostream>

#include <numeric>
#include <math.h>

#include <fmt/core.h>
#include <fmt/ostream.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/LaserScan.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/dist.h"
#include "psen_scan_v2/laserscan_validator.h"

namespace psen_scan_v2_test
{
std::map<int16_t, NormalDist> binsFromRosbag(std::string filepath)
{
  std::map<int16_t, NormalDist> bins;

  rosbag::Bag bag;
  bag.open(filepath, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("/laser_scanner/scan"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  std::for_each(view.begin(), view.end(), [&bins](const rosbag::MessageInstance& msg) {
    sensor_msgs::LaserScanConstPtr scan = msg.instantiate<sensor_msgs::LaserScan>();
    addScanToBin(scan, bins);
  });

  bag.close();

  return bins;
}

class ScanComparisonTests : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    ros::NodeHandle pnh{ "~" };

    std::string filepath;
    pnh.getParam("testfile", filepath);

    ROS_INFO_STREAM("Using testfile " << filepath);
    if (!boost::filesystem::exists(filepath))
    {
      ROS_ERROR_STREAM("File " << filepath << " not found!");
      FAIL();
    }

    bins_expected_ = binsFromRosbag(filepath);

    ASSERT_TRUE(pnh.getParam("test_duration", test_duration_));
  }

protected:
  static std::map<int16_t, NormalDist> bins_expected_;
  static int test_duration_;
};

std::map<int16_t, NormalDist> ScanComparisonTests::bins_expected_{};
int ScanComparisonTests::test_duration_{ 0 };

TEST_F(ScanComparisonTests, simpleCompare)
{
  ros::NodeHandle nh;

  size_t window_size = 120;  // Keep this high to avoid undersampling

  auto res = LaserScanValidator(nh, bins_expected_).validateScans(window_size, "/laser_scanner/scan", test_duration_);

  ASSERT_TRUE(res);
}

}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "scan_compare_test");

  // Needed since we use a subscriber
  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  return RUN_ALL_TESTS();
}
