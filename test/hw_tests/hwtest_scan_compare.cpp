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

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "psen_scan_v2/dist.h"
#include "psen_scan_v2/laserscan_validator.h"

namespace psen_scan_v2_test
{
using namespace std::chrono_literals;

typedef sensor_msgs::msg::LaserScan ScanType;
typedef std::shared_ptr<ScanType const> ScanConstPtr;

static const char* TESTDIR_ENV_VAR{ "HW_TEST_SCAN_COMPARE_TESTDIR" };
static const std::string TEST_DURATION_PARAM_NAME{ "test_duration" };

template <class SubscriptionT, class Rep, class Period>
::testing::AssertionResult isConnected(const std::shared_ptr<SubscriptionT>& subscription,
                                       const std::chrono::duration<Rep, Period>& timeout)
{
  auto start = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() - start < timeout)
  {
    if (subscription->get_publisher_count() > 0u)
    {
      return ::testing::AssertionSuccess();
    }
    rclcpp::sleep_for(100ns);
  }
  return ::testing::AssertionFailure() << "Failed to establish connection with publisher on topic"
                                       << subscription->get_topic_name();
}

class ScanComparisonTests : public ::testing::Test
{
public:
  void SetUp() override  // Omit using SetUpTestSuite() for googletest below v1.11.0, see
                         // https://github.com/google/googletest/issues/247
  {
    node_ptr_->declare_parameter(TEST_DURATION_PARAM_NAME);
    rclcpp::Parameter test_duration_param = node_ptr_->get_parameter(TEST_DURATION_PARAM_NAME);
    try
    {
      test_duration_ = test_duration_param.as_int();
    }
    catch (const rclcpp::ParameterTypeException& e)
    {
      FAIL() << e.what();
    }

    const char* path{ std::getenv(TESTDIR_ENV_VAR) };
    if (!path)
    {
      FAIL() << "Environment variable " << TESTDIR_ENV_VAR << " not set!";
    }

    RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Using test directory " << path);
    RCLCPP_INFO_STREAM(node_ptr_->get_logger(), "Using test duration " << test_duration_);

    try
    {
      bins_expected_ = binsFromRosbag(path);
    }
    catch (const std::runtime_error& e)
    {
      FAIL() << "Bag record in " << path
             << " could not be opened. Make sure the directory exists and the you have sufficient rights to open it.";
    }

    // Subscription callbacks have to be processed asynchronously during the test
    async_spinner_ = std::thread([this]() { rclcpp::spin(node_ptr_); });
  }

  void TearDown() override
  {
    rclcpp::shutdown();
    async_spinner_.join();
  }

protected:
  rclcpp::Node::SharedPtr node_ptr_{ std::make_shared<rclcpp::Node>("test_node") };
  std::thread async_spinner_;
  std::map<int16_t, NormalDist> bins_expected_{};
  int test_duration_{ 0 };
};

TEST_F(ScanComparisonTests, simpleCompare)
{
  size_t window_size = 120;  // Keep this high to avoid undersampling

  LaserScanValidator<ScanType> laser_scan_validator(bins_expected_);
  laser_scan_validator.reset();
  // Usage of std::bind is prevented by https://github.com/ros2/rclcpp/issues/273
  auto scan_subscription = node_ptr_->create_subscription<ScanType>(
      "/laser_1/scan", 1000, [&laser_scan_validator, &window_size](ScanConstPtr scan) {
        return laser_scan_validator.scanCb(scan, window_size, 0);
      });

  ASSERT_TRUE(isConnected(scan_subscription, 5s));

  EXPECT_TRUE(laser_scan_validator.waitForResult(test_duration_));
}

}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
