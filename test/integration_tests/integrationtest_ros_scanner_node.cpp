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

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_test;

using namespace ::testing;
using namespace psen_scan_v2_standalone::configuration;
using namespace psen_scan_v2_standalone_test;

namespace psen_scan_v2
{
#define Return_Future(promise_obj) ::testing::InvokeWithoutArgs([&promise_obj]() { return promise_obj.get_future(); })

#define EXPECT_SUCCESSFULL_START_WILL_OPEN_BARRIER(mock, barrier)                                                      \
  std::promise<void> start_promise_object;                                                                             \
  start_promise_object.set_value();                                                                                    \
  EXPECT_CALL(mock, start()).WillOnce(DoAll(OpenBarrier(barrier), Return_Future(start_promise_object)));

#define EXPECT_SUCCESSFULL_STOP_WILL_OPEN_BARRIER(mock, barrier)                                                       \
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

class ScanSubscriberMock
{
public:
  ScanSubscriberMock(ros::NodeHandle& nh);

  MOCK_METHOD1(callback, void(const sensor_msgs::LaserScan& msg));

private:
  ros::Subscriber subscriber_;
};

inline ScanSubscriberMock::ScanSubscriberMock(ros::NodeHandle& nh)
{
  subscriber_ = nh.subscribe("scan", QUEUE_SIZE, &ScanSubscriberMock::callback, this);
}

class ZoneSubscriberMock
{
public:
  ZoneSubscriberMock(ros::NodeHandle& nh);

  MOCK_METHOD1(callback, void(const std_msgs::UInt8& msg));

private:
  ros::Subscriber subscriber_;
};

inline ZoneSubscriberMock::ZoneSubscriberMock(ros::NodeHandle& nh)
{
  subscriber_ = nh.subscribe("active_zoneset", QUEUE_SIZE, &ZoneSubscriberMock::callback, this);
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

static LaserScan createValidLaserScan(const uint8_t active_zoneset=1)
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

TEST_F(RosScannerNodeTests, shouldPublishScansWhenLaserScanCallbackIsInvoked)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(
      nh_priv_, "scan", "scanner", configuration::DEFAULT_X_AXIS_ROTATION, scanner_config_);

  util::Barrier scan_topic_barrier;
  ScanSubscriberMock subscriber(nh_priv_);
  EXPECT_CALL(subscriber, callback(::testing::_))
      .WillOnce(testing::Return())
      .WillOnce(OpenBarrier(&scan_topic_barrier));

  util::Barrier start_barrier;
  util::Barrier stop_barrier;
  EXPECT_SUCCESSFULL_START_WILL_OPEN_BARRIER(ros_scanner_node.scanner_, &start_barrier);
  EXPECT_SUCCESSFULL_STOP_WILL_OPEN_BARRIER(ros_scanner_node.scanner_, &stop_barrier);

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

  util::Barrier scan_topic_barrier;
  ZoneSubscriberMock subscriber(nh_priv_);
  std_msgs::UInt8 expected_zone1;
  expected_zone1.data = 2;
  EXPECT_CALL(subscriber, callback(expected_zone1)).WillOnce(testing::Return());
  std_msgs::UInt8 expected_zone2;
  expected_zone2.data = 4;
  EXPECT_CALL(subscriber, callback(expected_zone2)).WillOnce(OpenBarrier(&scan_topic_barrier));

  util::Barrier start_barrier;
  util::Barrier stop_barrier;
  EXPECT_SUCCESSFULL_START_WILL_OPEN_BARRIER(ros_scanner_node.scanner_, &start_barrier);
  EXPECT_SUCCESSFULL_STOP_WILL_OPEN_BARRIER(ros_scanner_node.scanner_, &stop_barrier);

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });

  EXPECT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";

  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan(2));
  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan(4));

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
  EXPECT_SUCCESSFULL_START_WILL_OPEN_BARRIER(ros_scanner_node.scanner_, &start_barrier);

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
