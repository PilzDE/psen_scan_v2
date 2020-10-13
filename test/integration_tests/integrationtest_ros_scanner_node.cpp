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

#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <pilz_testutils/async_test.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/ros_scanner_node.h"
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/scanner_mock.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner.h"
#include "psen_scan_v2/scan_range.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_test;

using namespace ::testing;

namespace psen_scan_v2
{
#define Return_Future(promise_obj) ::testing::InvokeWithoutArgs([&promise_obj]() { return promise_obj.get_future(); })

static constexpr std::chrono::seconds LOOP_END_TIMEOUT{ 4 };

static constexpr int QUEUE_SIZE{ 10 };

static const std::string LASER_SCAN_RECEIVED{ "LASER_SCAN_RECEIVED" };
static const std::string SCANNER_STARTED{ "SCANNER_STARTED" };
static const std::string SCANNER_STOPPED{ "SCANNER_STOPPED" };

class SubscriberMock
{
public:
  void initialize(ros::NodeHandle& nh);

  MOCK_METHOD1(callback, void(const sensor_msgs::LaserScan& msg));

private:
  ros::Subscriber subscriber_;
};

inline void SubscriberMock::initialize(ros::NodeHandle& nh)
{
  subscriber_ = nh.subscribe("scan", QUEUE_SIZE, &SubscriberMock::callback, this);
}

static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr DefaultScanRange SCAN_RANGE{ TenthOfDegree(0), TenthOfDegree(2750) };
static constexpr int SCANNER_STARTED_TIMEOUT_MS{ 3000 };
static constexpr int SCANNER_STOPPED_TIMEOUT_MS{ 3000 };
static constexpr int LASERSCAN_RECEIVED_TIMEOUT{ 3000 };

class RosScannerNodeTests : public testing::Test, public testing::AsyncTest
{
protected:
  RosScannerNodeTests() : scanner_config_(HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, SCAN_RANGE){};
  ros::NodeHandle nh_priv_{ "~" };
  ScannerConfiguration scanner_config_;
};

TEST_F(RosScannerNodeTests, scannerInvocation)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", DEFAULT_X_AXIS_ROTATION, scanner_config_);

  std::promise<void> start_stop_barrier;
  start_stop_barrier.set_value();

  {
    ::testing::InSequence s;
    EXPECT_CALL(ros_scanner_node.scanner_, start())
        .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(SCANNER_STARTED), Return_Future(start_stop_barrier)));
    EXPECT_CALL(ros_scanner_node.scanner_, stop())
        .WillRepeatedly(DoAll(ACTION_OPEN_BARRIER_VOID(SCANNER_STOPPED), Return_Future(start_stop_barrier)));
  }

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  BARRIER(SCANNER_STARTED, SCANNER_STARTED_TIMEOUT_MS);
  ros_scanner_node.terminate();
  BARRIER(SCANNER_STOPPED, SCANNER_STOPPED_TIMEOUT_MS);
  EXPECT_EQ(loop.wait_for(LOOP_END_TIMEOUT), std::future_status::ready);
}

TEST_F(RosScannerNodeTests, scanTopicReceived)
{
  LaserScan laser_scan_fake(TenthOfDegree(1), TenthOfDegree(3), TenthOfDegree(5));
  laser_scan_fake.getMeasurements().push_back(1);

  SubscriberMock subscriber;
  EXPECT_CALL(subscriber, callback(::testing::_))
      .WillOnce(testing::Return())
      .WillOnce(ACTION_OPEN_BARRIER_VOID(LASER_SCAN_RECEIVED));

  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", DEFAULT_X_AXIS_ROTATION, scanner_config_);

  std::promise<void> start_stop_barrier;
  start_stop_barrier.set_value();
  EXPECT_CALL(ros_scanner_node.scanner_, start())
      .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(SCANNER_STARTED), Return_Future(start_stop_barrier)));

  subscriber.initialize(nh_priv_);
  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });

  BARRIER(SCANNER_STARTED, SCANNER_STARTED_TIMEOUT_MS);

  ros_scanner_node.scanner_.invokeLaserScanCallback(laser_scan_fake);
  ros_scanner_node.scanner_.invokeLaserScanCallback(laser_scan_fake);

  BARRIER(LASER_SCAN_RECEIVED, LASERSCAN_RECEIVED_TIMEOUT);
  ros_scanner_node.terminate();
  EXPECT_EQ(loop.wait_for(LOOP_END_TIMEOUT), std::future_status::ready);
}

TEST_F(RosScannerNodeTests, testMissingStopReply)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", DEFAULT_X_AXIS_ROTATION, scanner_config_);

  std::promise<void> start_barrier;
  start_barrier.set_value();

  // Stop barrier is unset, in other words the stop()-future object does not finish.
  std::promise<void> stop_barrier;

  {
    ::testing::InSequence s;
    EXPECT_CALL(ros_scanner_node.scanner_, start())
        .WillOnce(DoAll(ACTION_OPEN_BARRIER_VOID(SCANNER_STARTED), Return_Future(start_barrier)));
    EXPECT_CALL(ros_scanner_node.scanner_, stop()).WillOnce(Return_Future(stop_barrier));
  }

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
  BARRIER(SCANNER_STARTED, SCANNER_STARTED_TIMEOUT_MS);
  ros_scanner_node.terminate();
  EXPECT_EQ(loop.wait_for(LOOP_END_TIMEOUT), std::future_status::ready);
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
