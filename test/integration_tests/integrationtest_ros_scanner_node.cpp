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
#include <future>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt8.h>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scanner_config_builder.h"
#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/scan_range.h"
#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

#include "psen_scan_v2/laserscan_ros_conversions.h"
#include "psen_scan_v2/ros_scanner_node.h"

#include "psen_scan_v2/scanner_mock.h"
#include "psen_scan_v2/ros_integrationtest_helper.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_test;

using namespace ::testing;
using namespace psen_scan_v2_standalone::configuration;
using namespace psen_scan_v2_standalone_test;

namespace psen_scan_v2
{
#define Return_Future(promise_obj) ::testing::InvokeWithoutArgs([&promise_obj]() { return promise_obj.get_future(); })

#define EXPECT_SUCCESSFULL_START_AND_OPEN_BARRIER_ON_CALL(mock, barrier)                                               \
  std::promise<void> start_promise_object;                                                                             \
  start_promise_object.set_value();                                                                                    \
  EXPECT_CALL(mock, start()).WillOnce(DoAll(OpenBarrier(barrier), Return_Future(start_promise_object)));

#define EXPECT_SUCCESSFULL_STOP_AND_OPEN_BARRIER_ON_CALL(mock, barrier)                                                \
  std::promise<void> stop_promise_object;                                                                              \
  stop_promise_object.set_value();                                                                                     \
  EXPECT_CALL(mock, stop()).WillOnce(DoAll(OpenBarrier(barrier), Return_Future(stop_promise_object)));

static constexpr int QUEUE_SIZE{ 10 };

MATCHER_P(IsRosScanEqual, expected_ros_scan, "")
{
  const sensor_msgs::LaserScan actual_scan = arg;

  // clang-format off
  return (actual_scan.angle_min       == expected_ros_scan.angle_min) &&
         (actual_scan.angle_max       == expected_ros_scan.angle_max) &&
         (actual_scan.angle_increment == expected_ros_scan.angle_increment) &&
         (actual_scan.time_increment  == expected_ros_scan.time_increment) &&
         (actual_scan.scan_time       == expected_ros_scan.scan_time) &&
         (actual_scan.range_min       == expected_ros_scan.range_min) &&
         (actual_scan.ranges          == expected_ros_scan.ranges) &&
         (actual_scan.intensities     == expected_ros_scan.intensities);
  // clang-format on
}

class SubscriberMock
{
public:
  SubscriberMock(ros::NodeHandle& nh);

  MOCK_METHOD1(scan_callback, void(const sensor_msgs::LaserScan& msg));
  MOCK_METHOD1(zone_callback, void(const ros::MessageEvent<std_msgs::UInt8 const>& event));

private:
  ros::Subscriber scan_subscriber_;
  ros::Subscriber zone_subscriber_;
};

inline SubscriberMock::SubscriberMock(ros::NodeHandle& nh)
{
  scan_subscriber_ = nh.subscribe("scan", QUEUE_SIZE, &SubscriberMock::scan_callback, this);
  zone_subscriber_ = nh.subscribe("active_zoneset", QUEUE_SIZE, &SubscriberMock::zone_callback, this);
}

static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr ScanRange SCAN_RANGE{ util::TenthOfDegree(1), util::TenthOfDegree(2749) };
static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 3 };
static constexpr std::chrono::seconds LOOP_END_TIMEOUT{ 4 };
static constexpr std::chrono::seconds STOP_TIMEOUT{ 1 };

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

static LaserScan createValidLaserScan(const uint8_t active_zoneset = 1)
{
  psen_scan_v2_standalone::LaserScan laser_scan_fake(psen_scan_v2_standalone::util::TenthOfDegree(1),
                                                     psen_scan_v2_standalone::util::TenthOfDegree(3),
                                                     psen_scan_v2_standalone::util::TenthOfDegree(5),
                                                     14,
                                                     active_zoneset,
                                                     1000000000);
  laser_scan_fake.getMeasurements().push_back(1);
  return laser_scan_fake;
}

static std_msgs::UInt8 createActiveZonesetMsg(const uint8_t active_zoneset)
{
  std_msgs::UInt8 zone;
  zone.data = active_zoneset;
  return zone;
}

class RosScannerNodeTests : public testing::Test
{
protected:
  ros::NodeHandle nh_priv_{ "~" };
  ScannerConfiguration scanner_config_{ createValidConfig() };
};

TEST_F(RosScannerNodeTests, shouldStartAndStopSuccessfullyIfScannerRespondsToRequests)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  std::promise<void> hw_finished_request;
  hw_finished_request.set_value();
  util::Barrier start_barrier;
  util::Barrier stop_barrier;

  {
    ::testing::InSequence s;
    EXPECT_CALL(ros_scanner_node.scanner_, start())
        .WillOnce(DoAll(OpenBarrier(&start_barrier), Return_Future(hw_finished_request)));
    EXPECT_CALL(ros_scanner_node.scanner_, stop())
        .WillOnce(DoAll(OpenBarrier(&stop_barrier), Return_Future(hw_finished_request)));
  }

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  EXPECT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";
  ros_scanner_node.terminate();
  EXPECT_BARRIER_OPENS(stop_barrier, DEFAULT_TIMEOUT) << "Scanner stop was not called";
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldThrowExceptionSetInScannerStartFuture)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  std::promise<void> hw_finished_request;
  const std::string error_msg = "error msg for testing";
  hw_finished_request.set_exception(std::make_exception_ptr(std::runtime_error(error_msg)));

  ON_CALL(ros_scanner_node.scanner_, start()).WillByDefault(Return_Future(hw_finished_request));
  EXPECT_THROW_AND_WHAT(ros_scanner_node.run(), std::runtime_error, error_msg.c_str());
}

TEST_F(RosScannerNodeTests, shouldThrowDelayedExceptionSetInScannerStartFuture)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  util::Barrier start_barrier;
  std::promise<void> hw_finished_request;
  ON_CALL(ros_scanner_node.scanner_, start())
      .WillByDefault(DoAll(OpenBarrier(&start_barrier), Return_Future(hw_finished_request)));

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  start_barrier.waitTillRelease(DEFAULT_TIMEOUT);
  ros::Duration(1.0).sleep();

  const std::string error_msg = "error msg for testing";
  hw_finished_request.set_exception(std::make_exception_ptr(std::runtime_error(error_msg)));

  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
  EXPECT_THROW_AND_WHAT(loop.get(), std::runtime_error, error_msg.c_str());
}

TEST_F(RosScannerNodeTests, shouldProvideScanTopic)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  util::Barrier start_barrier;
  util::Barrier stop_barrier;
  EXPECT_SUCCESSFULL_START_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &start_barrier);
  EXPECT_SUCCESSFULL_STOP_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &stop_barrier);

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });

  EXPECT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";
  EXPECT_TRUE(TopicExists("/integrationtest_ros_scanner_node/scan"));
  ros_scanner_node.terminate();
  EXPECT_BARRIER_OPENS(stop_barrier, DEFAULT_TIMEOUT) << "Scanner stop was not called";
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldProvideActiveZonesetTopic)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  util::Barrier start_barrier;
  util::Barrier stop_barrier;
  EXPECT_SUCCESSFULL_START_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &start_barrier);
  EXPECT_SUCCESSFULL_STOP_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &stop_barrier);

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });

  EXPECT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";
  EXPECT_TRUE(TopicExists("/integrationtest_ros_scanner_node/active_zoneset"));
  ros_scanner_node.terminate();
  EXPECT_BARRIER_OPENS(stop_barrier, DEFAULT_TIMEOUT) << "Scanner stop was not called";
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldPublishScansWhenLaserScanCallbackIsInvoked)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  util::Barrier scan_topic_barrier;
  SubscriberMock subscriber(nh_priv_);
  EXPECT_CALL(subscriber, scan_callback(::testing::_))
      .WillOnce(testing::Return())
      .WillOnce(OpenBarrier(&scan_topic_barrier));

  util::Barrier start_barrier;
  util::Barrier stop_barrier;
  EXPECT_SUCCESSFULL_START_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &start_barrier);
  EXPECT_SUCCESSFULL_STOP_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &stop_barrier);

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });

  EXPECT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";

  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan());
  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan());

  EXPECT_BARRIER_OPENS(scan_topic_barrier, DEFAULT_TIMEOUT) << "Scan message was not sended";
  ros_scanner_node.terminate();
  EXPECT_BARRIER_OPENS(stop_barrier, DEFAULT_TIMEOUT) << "Scanner stop was not called";
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldPublishActiveZonesetWhenLaserScanCallbackIsInvoked)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  const uint8_t first_zone{ 2 };
  const uint8_t second_zone{ 4 };

  util::Barrier scan_topic_barrier;
  SubscriberMock subscriber(nh_priv_);
  EXPECT_CALL(subscriber, zone_callback(messageEQ(createActiveZonesetMsg(first_zone))));
  EXPECT_CALL(subscriber, zone_callback(messageEQ(createActiveZonesetMsg(second_zone))))
      .WillOnce(OpenBarrier(&scan_topic_barrier));

  util::Barrier start_barrier;
  util::Barrier stop_barrier;
  EXPECT_SUCCESSFULL_START_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &start_barrier);
  EXPECT_SUCCESSFULL_STOP_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &stop_barrier);

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });

  EXPECT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";

  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan(first_zone));
  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan(second_zone));

  EXPECT_BARRIER_OPENS(scan_topic_barrier, DEFAULT_TIMEOUT) << "Scan message was not sended";
  ros_scanner_node.terminate();
  EXPECT_BARRIER_OPENS(stop_barrier, DEFAULT_TIMEOUT) << "Scanner stop was not called";
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldReceiveLatchedActiveZonesetMsg)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);
  util::Barrier start_barrier;
  util::Barrier stop_barrier;
  EXPECT_SUCCESSFULL_START_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &start_barrier);
  EXPECT_SUCCESSFULL_STOP_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &stop_barrier);

  const uint8_t first_zone{ 2 };

  util::Barrier scan_topic_barrier;
  SubscriberMock subscriber(nh_priv_);
  EXPECT_CALL(subscriber, zone_callback(AllOf(isLatched(), messageEQ(createActiveZonesetMsg(first_zone)))))
      .WillOnce(OpenBarrier(&scan_topic_barrier));

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  EXPECT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";

  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan(first_zone));

  EXPECT_BARRIER_OPENS(scan_topic_barrier, DEFAULT_TIMEOUT) << "Scan message was not sended";
  ros_scanner_node.terminate();
  EXPECT_BARRIER_OPENS(stop_barrier, DEFAULT_TIMEOUT) << "Scanner stop was not called";
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldWaitWhenStopRequestResponseIsMissing)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  util::Barrier start_barrier;
  EXPECT_SUCCESSFULL_START_AND_OPEN_BARRIER_ON_CALL(ros_scanner_node.scanner_, &start_barrier);

  std::promise<void> unset_promise;
  EXPECT_CALL(ros_scanner_node.scanner_, stop()).WillOnce(Return_Future(unset_promise));

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  EXPECT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";
  ros_scanner_node.terminate();
  EXPECT_FUTURE_TIMEOUT(loop, STOP_TIMEOUT) << "ROS node did not wait for stop response";
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

}  // namespace psen_scan_v2

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_ros_scanner_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
