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

#include <ros/ros.h>
#include <gtest/gtest.h>

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include <map>
#include <string>

#include <sensor_msgs/LaserScan.h>

#include "psen_scan_v2/dist.h"
#include "psen_scan_v2/laserscan_validator.h"

namespace psen_scan_v2_test
{
typedef sensor_msgs::LaserScan ScanType;
typedef boost::shared_ptr<ScanType const> ScanConstPtr;

static constexpr int32_t WAIT_FOR_MESSAGE_TIMEOUT_S{ 5 };

class ScanComparisonTests : public ::testing::Test
{
public:
  void SetUp() override  // Omit using SetUpTestSuite() for googletest below v1.11.0, see
                         // https://github.com/google/googletest/issues/247
  {
    ros::NodeHandle pnh{ "~" };

    std::string filepath;
    pnh.getParam("testfile", filepath);

    ROS_INFO_STREAM("Using testfile " << filepath);

    try
    {
      bins_expected_ = binsFromRosbag(filepath);
    }
    catch (const rosbag::BagIOException& e)
    {
      FAIL() << "File " << filepath
             << " could not be opened. Make sure the file exists and the you have sufficient rights to open it.";
    }
    catch (const rosbag::BagException& e)
    {
      FAIL() << "There was an error opening " << filepath;
    }

    ASSERT_TRUE(pnh.getParam("test_duration", test_duration_));
  }

protected:
  std::map<int16_t, NormalDist> bins_expected_{};
  int test_duration_{ 0 };
};

TEST_F(ScanComparisonTests, simpleCompare)
{
  ros::NodeHandle nh;

  size_t window_size = 120;  // Keep this high to avoid undersampling

  LaserScanValidator<ScanType> laser_scan_validator(bins_expected_);
  laser_scan_validator.reset();
  auto scan_subscriber = nh.subscribe<ScanType>(
      "/laser_1/scan",
      1000,
      boost::bind(&LaserScanValidator<ScanType>::scanCb, &laser_scan_validator, boost::placeholders::_1, window_size));

  ros::topic::waitForMessage<ScanType>("/laser_1/scan", ros::Duration(WAIT_FOR_MESSAGE_TIMEOUT_S, 0));
  ASSERT_EQ(1, scan_subscriber.getNumPublishers())
      << "Failed to establish connection with publisher on laserscan-topic";

  ASSERT_TRUE(laser_scan_validator.waitForResult(test_duration_));
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
