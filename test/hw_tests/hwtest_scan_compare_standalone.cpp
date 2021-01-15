// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#include <map>
#include <string>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include "psen_scan_v2_standalone/core.h"

#include "psen_scan_v2/dist.h"
#include "psen_scan_v2/laserscan_validator.h"

using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_test;

namespace psen_scan_v2_standalone_test
{
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
    ASSERT_TRUE(pnh.getParam("host_ip", host_ip_));
    ASSERT_TRUE(pnh.getParam("scanner_ip", scanner_ip_));

    double angle_start;
    double angle_end;
    ASSERT_TRUE(pnh.getParam("angle_start", angle_start));
    ASSERT_TRUE(pnh.getParam("angle_end", angle_end));
    angle_start_ = TenthOfDegree(degreeToTenthDegree(angle_start));
    angle_end_ = TenthOfDegree(degreeToTenthDegree(angle_end));
  }

protected:
  static std::map<int16_t, NormalDist> bins_expected_;
  static int test_duration_;
  static TenthOfDegree angle_start_;
  static TenthOfDegree angle_end_;
  static std::string host_ip_;
  static std::string scanner_ip_;
};

std::map<int16_t, NormalDist> ScanComparisonTests::bins_expected_{};
int ScanComparisonTests::test_duration_{ 0 };
TenthOfDegree ScanComparisonTests::angle_start_{ 0 };
TenthOfDegree ScanComparisonTests::angle_end_{ 0 };
std::string ScanComparisonTests::host_ip_{};
std::string ScanComparisonTests::scanner_ip_{};

TEST_F(ScanComparisonTests, simpleCompare)
{
  size_t window_size = 120;  // Keep this high to avoid undersampling

  LaserScanValidator<LaserScan> laser_scan_validator(bins_expected_);
  laser_scan_validator.reset();

  setLogLevel(CONSOLE_BRIDGE_LOG_WARN);

  DefaultScanRange scan_range{ angle_start_, angle_end_ };

  ScannerConfigurationBuilder config_builder;
  config_builder.hostIP(host_ip_).scannerIp(scanner_ip_).scanRange(scan_range);

  ScannerV2 scanner(config_builder.build(), [&laser_scan_validator, &window_size](const LaserScan& scan) {
    return laser_scan_validator.scanCb(boost::make_shared<LaserScan const>(scan), window_size);
  });
  scanner.start();

  EXPECT_TRUE(laser_scan_validator.waitForResult(test_duration_));

  scanner.stop();
}
}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "scan_compare_standalone_test");

  return RUN_ALL_TESTS();
}
