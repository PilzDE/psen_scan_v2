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

#include <memory>
#include <chrono>
#include <thread>
#include <functional>
#include <future>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scanner_config_builder.h"
#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/scan_range.h"
#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

#include "psen_scan_v2/laserscan_ros_conversions.h"
#include "psen_scan_v2/ros_scanner_node.h"

#include "psen_scan_v2/ros_barrier.h"
#include "psen_scan_v2/scanner_mock.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_test;

using namespace ::testing;
using namespace psen_scan_v2_standalone::configuration;
using namespace psen_scan_v2_standalone_test;

namespace psen_scan_v2
{
#define Return_Future(promise_obj) ::testing::InvokeWithoutArgs([&promise_obj]() { return promise_obj.get_future(); })

static constexpr int QUEUE_SIZE{ 10 };
static const std::string SCANNER_PREFIX{ "scanner" };
static const std::string SCAN_TOPIC_NAME{ "scan" };

class ScanSubscriberMock : public rclcpp::Node
{
public:
  ScanSubscriberMock();
  MOCK_CONST_METHOD1(callback, void(sensor_msgs::msg::LaserScan::ConstSharedPtr msg));

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

inline ScanSubscriberMock::ScanSubscriberMock() : rclcpp::Node("scan_subscriber", SCANNER_PREFIX)
{
  subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      SCAN_TOPIC_NAME, QUEUE_SIZE, std::bind(&ScanSubscriberMock::callback, this, std::placeholders::_1));
}

static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr ScanRange SCAN_RANGE{ util::TenthOfDegree(1), util::TenthOfDegree(2749) };
static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 3 };
static constexpr std::chrono::seconds LOOP_END_TIMEOUT{ 4 };

static ScannerConfiguration createValidConfig()
{
  return ScannerConfigurationBuilder()
      .hostIP(HOST_IP)
      .hostDataPort(HOST_UDP_PORT_DATA)
      .hostControlPort(HOST_UDP_PORT_CONTROL)
      .scannerIp(DEVICE_IP)
      .scannerDataPort(configuration::DATA_PORT_OF_SCANNER_DEVICE)
      .scannerControlPort(configuration::CONTROL_PORT_OF_SCANNER_DEVICE)
      .scanRange(SCAN_RANGE)
      .build();
}

class RosScannerNodeTests : public testing::Test
{
protected:
  rclcpp::Node::SharedPtr node_ptr_{ std::make_shared<rclcpp::Node>("test_node") };
  ScannerConfiguration scanner_config_{ createValidConfig() };
};

TEST_F(RosScannerNodeTests, testScannerInvocation)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      node_ptr_, SCAN_TOPIC_NAME, SCANNER_PREFIX, configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  std::promise<void> start_stop_barrier;
  start_stop_barrier.set_value();
  util::Barrier start_barrier;
  util::Barrier stop_barrier;

  {
    ::testing::InSequence s;
    EXPECT_CALL(ros_scanner_node.scanner_, start())
        .WillOnce(DoAll(OpenBarrier(&start_barrier), Return_Future(start_stop_barrier)));
    EXPECT_CALL(ros_scanner_node.scanner_, stop())
        .WillOnce(DoAll(OpenBarrier(&stop_barrier), Return_Future(start_stop_barrier)));
  }

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  EXPECT_TRUE(start_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Scanner start was not called";
  ros_scanner_node.terminate();
  EXPECT_TRUE(stop_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Scanner stop was not called";
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, testScanTopicReceived)
{
  LaserScan laser_scan_fake(util::TenthOfDegree(1), util::TenthOfDegree(3), util::TenthOfDegree(5), 14, 1000000000);
  laser_scan_fake.getMeasurements().push_back(1);

  RosBarrier scan_topic_barrier;
  auto subscriber = std::make_shared<ScanSubscriberMock>();
  EXPECT_CALL(*subscriber, callback(::testing::_)).WillOnce(OpenBarrier(&scan_topic_barrier));

  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      node_ptr_, SCAN_TOPIC_NAME, SCANNER_PREFIX, configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  std::promise<void> start_stop_barrier;
  start_stop_barrier.set_value();
  util::Barrier start_barrier;
  EXPECT_CALL(ros_scanner_node.scanner_, start())
      .WillOnce(DoAll(OpenBarrier(&start_barrier), Return_Future(start_stop_barrier)));

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });

  EXPECT_TRUE(start_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Scanner start was not called";

  ros_scanner_node.scanner_.invokeLaserScanCallback(laser_scan_fake);

  EXPECT_EQ(rclcpp::FutureReturnCode::SUCCESS, scan_topic_barrier.spinUntilRelease(subscriber, DEFAULT_TIMEOUT))
      << "Scan message was not sended";
  ros_scanner_node.terminate();
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, testMissingStopReply)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      node_ptr_, SCAN_TOPIC_NAME, SCANNER_PREFIX, configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  std::promise<void> start_barrier;
  start_barrier.set_value();

  // Stop barrier is unset, in other words the stop()-future object does not finish.
  std::promise<void> stop_barrier;
  util::Barrier start_async_barrier;

  {
    ::testing::InSequence s;
    EXPECT_CALL(ros_scanner_node.scanner_, start())
        .WillOnce(DoAll(OpenBarrier(&start_async_barrier), Return_Future(start_barrier)));
    EXPECT_CALL(ros_scanner_node.scanner_, stop()).WillOnce(Return_Future(stop_barrier));
  }

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  EXPECT_TRUE(start_async_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Scanner start was not called";
  ros_scanner_node.terminate();
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

}  // namespace psen_scan_v2

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
