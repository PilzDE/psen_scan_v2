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
#include "psen_scan_v2_standalone/util/assertions.h"
#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

#include "psen_scan_v2/laserscan_ros_conversions.h"
#include "psen_scan_v2/ros_scanner_node.h"

#include "psen_scan_v2/scanner_mock.h"
#include "psen_scan_v2/ros_integrationtest_helper.h"

using namespace ::testing;
using namespace psen_scan_v2_test;
namespace standalone = psen_scan_v2_standalone;
namespace standalone_test = psen_scan_v2_standalone_test;
namespace util = psen_scan_v2_standalone::util;

namespace psen_scan_v2
{
static constexpr int QUEUE_SIZE{ 10 };

class SubscriberMock
{
public:
  SubscriberMock(ros::NodeHandle& nh);

  MOCK_METHOD1(scan_callback, void(const sensor_msgs::LaserScan& msg));
  MOCK_METHOD1(zone_callback, void(const std_msgs::UInt8& msg));
  MOCK_METHOD1(io_callback, void(const IOState& msg));

private:
  ros::Subscriber scan_subscriber_;
  ros::Subscriber zone_subscriber_;
  ros::Subscriber io_subscriber_;
};

inline SubscriberMock::SubscriberMock(ros::NodeHandle& nh)
{
  scan_subscriber_ = nh.subscribe("scan", QUEUE_SIZE, &SubscriberMock::scan_callback, this);
  zone_subscriber_ = nh.subscribe("active_zoneset", QUEUE_SIZE, &SubscriberMock::zone_callback, this);
  io_subscriber_ = nh.subscribe("io_state", QUEUE_SIZE, &SubscriberMock::io_callback, this);
}

static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr standalone::ScanRange SCAN_RANGE{ util::TenthOfDegree(1), util::TenthOfDegree(2749) };
static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 3 };
static constexpr std::chrono::seconds LOOP_END_TIMEOUT{ 4 };
static constexpr std::chrono::seconds STOP_TIMEOUT{ 1 };


static const standalone::LaserScan::IOData IO_DATA1{
  { standalone::IOState({ standalone::PinState(1, "input0", true) }, {}, { standalone::PinState(14, "output", false) }),
    standalone::IOState({},
                        { standalone::PinState(21, "logical", true), standalone::PinState(3, "logical2", false) },
                        {}) }
};
static const standalone::LaserScan::IOData IO_DATA2{ {
    standalone::IOState({ standalone::PinState(3, "input0", false) }, {}, {}),
} };

static void setDefaultActions(ScannerMock& mock, util::Barrier& start_barrier)
{
  ON_CALL(mock, start()).WillByDefault(DoAll(standalone_test::OpenBarrier(&start_barrier), ReturnReadyVoidFuture()));
  ON_CALL(mock, stop()).WillByDefault(ReturnReadyVoidFuture());
}

static standalone::ScannerConfiguration createValidConfig()
{
  return standalone::ScannerConfigurationBuilder(DEVICE_IP)
      .hostIP(HOST_IP)
      .hostDataPort(HOST_UDP_PORT_DATA)
      .hostControlPort(HOST_UDP_PORT_CONTROL)
      .scannerDataPort(standalone::configuration::DATA_PORT_OF_SCANNER_DEVICE)
      .scannerControlPort(standalone::configuration::CONTROL_PORT_OF_SCANNER_DEVICE)
      .scanRange(SCAN_RANGE);
}

static standalone::LaserScan createValidLaserScan(const uint8_t active_zoneset = 1)
{
  standalone::LaserScan laser_scan_fake(
      util::TenthOfDegree(1), util::TenthOfDegree(3), util::TenthOfDegree(5), 14, active_zoneset, 1000000000);
  laser_scan_fake.setMeasurements({ 0.1, 2.31 });
  laser_scan_fake.setIntensities({ 1.0, 0.3 });
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
  standalone::ScannerConfiguration scanner_config_{ createValidConfig() };
};

TEST_F(RosScannerNodeTests, shouldStartAndStopSuccessfullyIfScannerRespondsToRequests)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", 1.0 /*x_axis_rotation*/, scanner_config_);

  util::Barrier start_barrier;
  util::Barrier stop_barrier;

  {
    InSequence s;
    EXPECT_CALL(ros_scanner_node.scanner_, start())
        .WillOnce(DoAll(standalone_test::OpenBarrier(&start_barrier), ReturnReadyVoidFuture()));
    EXPECT_CALL(ros_scanner_node.scanner_, stop())
        .WillOnce(DoAll(standalone_test::OpenBarrier(&stop_barrier), ReturnReadyVoidFuture()));
  }

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  start_barrier.waitTillRelease(DEFAULT_TIMEOUT);

  ros_scanner_node.terminate();
  stop_barrier.waitTillRelease(DEFAULT_TIMEOUT);
  EXPECT_FUTURE_IS_READY(loop, LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldThrowExceptionSetInScannerStartFuture)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", 1.0 /*x_axis_rotation*/, scanner_config_);

  std::promise<void> hw_finished_request;
  const std::string error_msg = "error msg for testing";
  hw_finished_request.set_exception(std::make_exception_ptr(std::runtime_error(error_msg)));

  ON_CALL(ros_scanner_node.scanner_, start()).WillByDefault(ReturnFuture(&hw_finished_request));
  EXPECT_THROW_AND_WHAT(ros_scanner_node.run(), std::runtime_error, error_msg.c_str());
}

TEST_F(RosScannerNodeTests, shouldThrowDelayedExceptionSetInScannerStartFuture)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", 1.0 /*x_axis_rotation*/, scanner_config_);

  util::Barrier start_barrier;
  std::promise<void> hw_finished_request;
  ON_CALL(ros_scanner_node.scanner_, start())
      .WillByDefault(DoAll(standalone_test::OpenBarrier(&start_barrier), ReturnFuture(&hw_finished_request)));

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
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", 1.0 /*x_axis_rotation*/, scanner_config_);
  EXPECT_TRUE(TopicExists("/integrationtest_ros_scanner_node/scan"));
}

TEST_F(RosScannerNodeTests, shouldProvideActiveZonesetTopic)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", 1.0 /*x_axis_rotation*/, scanner_config_);
  EXPECT_TRUE(TopicExists("/integrationtest_ros_scanner_node/active_zoneset"));
}

TEST_F(RosScannerNodeTests, shouldProvideIOTopic)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", 1.0 /*x_axis_rotation*/, scanner_config_);
  EXPECT_TRUE(TopicExists("/integrationtest_ros_scanner_node/io_state"));
}

TEST_F(RosScannerNodeTests, shouldPublishScansWhenLaserScanCallbackIsInvoked)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", 1.0 /*x_axis_rotation*/, scanner_config_);

  util::Barrier scan_topic_barrier;
  SubscriberMock subscriber(nh_priv_);
  EXPECT_CALL(subscriber, scan_callback(::testing::_))
      .WillOnce(Return())
      .WillOnce(standalone_test::OpenBarrier(&scan_topic_barrier));

  util::Barrier start_barrier;
  setDefaultActions(ros_scanner_node.scanner_, start_barrier);

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  ASSERT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";

  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan());
  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan());
  scan_topic_barrier.waitTillRelease(DEFAULT_TIMEOUT);

  ros_scanner_node.terminate();
  loop.wait_for(LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldPublishActiveZonesetWhenLaserScanCallbackIsInvoked)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", 1.0 /*x_axis_rotation*/, scanner_config_);

  const uint8_t first_zone{ 2 };
  const uint8_t second_zone{ 4 };

  util::Barrier zoneset_topic_barrier;
  SubscriberMock subscriber(nh_priv_);
  {
    InSequence s;
    EXPECT_CALL(subscriber, zone_callback(createActiveZonesetMsg(first_zone))).Times(1);
    EXPECT_CALL(subscriber, zone_callback(createActiveZonesetMsg(second_zone)))
        .WillOnce(standalone_test::OpenBarrier(&zoneset_topic_barrier));
  }

  util::Barrier start_barrier;
  setDefaultActions(ros_scanner_node.scanner_, start_barrier);

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  ASSERT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";

  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan(first_zone));
  ros_scanner_node.scanner_.invokeLaserScanCallback(createValidLaserScan(second_zone));
  zoneset_topic_barrier.waitTillRelease(DEFAULT_TIMEOUT);

  ros_scanner_node.terminate();
  loop.wait_for(LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldPublishIOStatesEqualToConversionOfSuppliedStandaloneIOStates)
{
  const std::string prefix{ "scanner" };
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", prefix, 1.0 /*x_axis_rotation*/, scanner_config_);

  auto scan = createValidLaserScan();

  util::Barrier io_topic_barrier;
  SubscriberMock subscriber(nh_priv_);
  {
    InSequence s;
    EXPECT_CALL(subscriber, io_callback(IOStateMsgEq(toIOStateMsg(IO_DATA1.at(0), prefix, scan.getTimestamp()))))
        .Times(1);
    EXPECT_CALL(subscriber, io_callback(IOStateMsgEq(toIOStateMsg(IO_DATA1.at(1), prefix, scan.getTimestamp()))))
        .Times(1);
    EXPECT_CALL(subscriber, io_callback(IOStateMsgEq(toIOStateMsg(IO_DATA2.at(0), prefix, scan.getTimestamp()))))
        .WillOnce(standalone_test::OpenBarrier(&io_topic_barrier));
  }

  util::Barrier start_barrier;
  setDefaultActions(ros_scanner_node.scanner_, start_barrier);

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  ASSERT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";

  scan.setIOStates(IO_DATA1);
  ros_scanner_node.scanner_.invokeLaserScanCallback(scan);
  scan.setIOStates(IO_DATA2);
  ros_scanner_node.scanner_.invokeLaserScanCallback(scan);
  io_topic_barrier.waitTillRelease(DEFAULT_TIMEOUT);

  ros_scanner_node.terminate();
  loop.wait_for(LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldPublishScanEqualToConversionOfSuppliedLaserScan)
{
  const std::string prefix = "scanner";
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", prefix, 1.0 /*x_axis_rotation*/, scanner_config_);

  util::Barrier start_barrier;
  setDefaultActions(ros_scanner_node.scanner_, start_barrier);

  util::Barrier scan_topic_barrier;
  SubscriberMock subscriber(nh_priv_);
  const auto scan = createValidLaserScan();
  EXPECT_CALL(subscriber, scan_callback(LaserScanMsgEq(toLaserScanMsg(scan, prefix, 1.0 /*x_axis_rotation*/))))
      .WillOnce(standalone_test::OpenBarrier(&scan_topic_barrier));

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });

  ASSERT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";

  ros_scanner_node.scanner_.invokeLaserScanCallback(scan);
  scan_topic_barrier.waitTillRelease(DEFAULT_TIMEOUT);

  ros_scanner_node.terminate();
  loop.wait_for(LOOP_END_TIMEOUT);
}

TEST_F(RosScannerNodeTests, shouldWaitWhenStopRequestResponseIsMissing)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", 1.0 /*x_axis_rotation*/, scanner_config_);

  util::Barrier start_barrier;
  ON_CALL(ros_scanner_node.scanner_, start())
      .WillByDefault(DoAll(standalone_test::OpenBarrier(&start_barrier), ReturnReadyVoidFuture()));

  std::promise<void> p;
  ON_CALL(ros_scanner_node.scanner_, stop()).WillByDefault(ReturnFuture(&p));

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  ASSERT_BARRIER_OPENS(start_barrier, DEFAULT_TIMEOUT) << "Scanner start was not called";

  ros_scanner_node.terminate();
  EXPECT_FUTURE_TIMEOUT(loop, STOP_TIMEOUT) << "ROS node did not wait for stop response";
  loop.wait_for(LOOP_END_TIMEOUT);
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
