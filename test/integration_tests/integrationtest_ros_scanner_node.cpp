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

#include "psen_scan_v2/degree_to_rad.h"
#include "psen_scan_v2/ros_scanner_node.h"
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/scanner_mock.h"
#include "psen_scan_v2/scanner_configuration.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_test;
using ::testing::DoAll;
using ::testing::Return;

namespace psen_scan_v2
{
static constexpr std::chrono::seconds LOOP_END_TIMEOUT{ 3 };

static constexpr int QUEUE_SIZE{ 10 };

static const std::string LASER_SCAN_RECEIVED{ "LASER_SCAN_RECEIVED" };
static const std::string SCANNER_STARTED{ "SCANNER_STARTED" };
static const std::string SCANNER_STOPPED{ "SCANNER_STOPPED" };

static constexpr double DEFAULT_X_AXIS_ROTATION{ degreeToRad(137.5) };

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
static constexpr double START_ANGLE{ 0. };
static constexpr double END_ANGLE{ 275. * 2 * M_PI / 360. };

class RosScannerNodeTests : public testing::Test, public testing::AsyncTest
{
protected:
  RosScannerNodeTests()
    : scanner_config_(HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, START_ANGLE, END_ANGLE){};
  ros::NodeHandle nh_priv_{ "~" };
  ScannerConfiguration scanner_config_;
};

TEST_F(RosScannerNodeTests, testScannerInvocation)
{
  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", DEFAULT_X_AXIS_ROTATION, scanner_config_);

  LaserScan laser_scan_fake(0.02, 0.03, 0.05);
  laser_scan_fake.getMeasurements().push_back(1);

  {
    ::testing::InSequence s;
    EXPECT_CALL(ros_scanner_node.scanner_, start()).WillOnce(ACTION_OPEN_BARRIER_VOID(SCANNER_STARTED));
    EXPECT_CALL(ros_scanner_node.scanner_, getCompleteScan()).WillRepeatedly(Return(laser_scan_fake));
    EXPECT_CALL(ros_scanner_node.scanner_, stop()).WillOnce(ACTION_OPEN_BARRIER_VOID(SCANNER_STOPPED));
  }

  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.processingLoop(); });
  BARRIER(SCANNER_STARTED);
  ros_scanner_node.terminateProcessingLoop();
  BARRIER(SCANNER_STOPPED);
  EXPECT_EQ(loop.wait_for(LOOP_END_TIMEOUT), std::future_status::ready);
}

TEST_F(RosScannerNodeTests, testScanTopicReceived)
{
  SubscriberMock subscriber;
  EXPECT_CALL(subscriber, callback(::testing::_)).WillOnce(ACTION_OPEN_BARRIER_VOID(LASER_SCAN_RECEIVED));

  LaserScan laser_scan_fake(0.02, 0.03, 0.05);
  laser_scan_fake.getMeasurements().push_back(1);

  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", DEFAULT_X_AXIS_ROTATION, scanner_config_);
  EXPECT_CALL(ros_scanner_node.scanner_, getCompleteScan()).WillRepeatedly(Return(laser_scan_fake));

  subscriber.initialize(nh_priv_);
  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.processingLoop(); });
  BARRIER(LASER_SCAN_RECEIVED);
  ros_scanner_node.terminateProcessingLoop();
  EXPECT_EQ(loop.wait_for(LOOP_END_TIMEOUT), std::future_status::ready);
}

ACTION(ThrowScanBuildFailure)
{
  throw LaserScanBuildFailure();
}

TEST_F(RosScannerNodeTests, testScanBuildFailure)
{
  SubscriberMock subscriber;
  EXPECT_CALL(subscriber, callback(::testing::_)).Times(1).WillOnce(ACTION_OPEN_BARRIER_VOID(LASER_SCAN_RECEIVED));

  LaserScan laser_scan_fake(0.02, 0.03, 0.05);
  laser_scan_fake.getMeasurements().push_back(1);

  ROSScannerNodeT<ScannerMock> ros_scanner_node(nh_priv_, "scan", "scanner", DEFAULT_X_AXIS_ROTATION, scanner_config_);
  {
    ::testing::InSequence s;
    EXPECT_CALL(ros_scanner_node.scanner_, getCompleteScan())
        .Times(100)
        .WillRepeatedly(DoAll(ThrowScanBuildFailure(), Return(laser_scan_fake)));
    EXPECT_CALL(ros_scanner_node.scanner_, getCompleteScan()).Times(1).WillRepeatedly(Return(laser_scan_fake));
  }

  subscriber.initialize(nh_priv_);
  std::future<void> loop = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.processingLoop(); });
  BARRIER(LASER_SCAN_RECEIVED);
  ros_scanner_node.terminateProcessingLoop();
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
