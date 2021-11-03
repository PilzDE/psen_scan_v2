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

#include <chrono>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

#include "psen_scan_v2/ros_integrationtest_helper.h"

#include "psen_scan_v2_standalone/util/expectations.h"

namespace psen_scan_v2_test
{
using namespace std::chrono_literals;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_standalone_test;

MATCHER(hasPoints, "")
{
  return !arg.points.empty();
}

MATCHER(IsTriangleList, "")
{
  return arg.type == visualization_msgs::Marker::TRIANGLE_LIST;
}

MATCHER_P(matchesName, expectedName, "")
{
  return arg.ns == expectedName;
}

class SubscriberMock
{
public:
  SubscriberMock()
  {
    subscriber_ = nh_.subscribe("/test_ns_laser_1/active_zoneset_marker", 10, &SubscriberMock::callback, this);
  }

  MOCK_METHOD1(callback, void(const visualization_msgs::Marker& event));

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};

class ActiveZonesetNodeTest : public testing::Test
{
public:
  void SetUp() override
  {
    ros::NodeHandle nh;
    pub_active_ = nh.advertise<std_msgs::UInt8>("/test_ns_laser_1/active_zoneset", 1, true);
  }
  void sendActiveZone(uint8_t zone);

public:
  ros::Publisher pub_active_;
};

void ActiveZonesetNodeTest::sendActiveZone(uint8_t zone)
{
  std_msgs::UInt8 active_zone_msg;
  active_zone_msg.data = zone;
  pub_active_.publish(active_zone_msg);
}

TEST_F(ActiveZonesetNodeTest, shouldAdvertiseZonesetMarkerTopic)
{
  EXPECT_TRUE(TopicExists("/test_ns_laser_1/active_zoneset_marker"));
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkerWithCorrectType)
{
  SubscriberMock subscriber_mock;
  EXPECT_CALLS_ON_STATEMENT_AND_WAIT(subscriber_mock, callback(IsTriangleList()), sendActiveZone(0), 3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkerWithPoints)
{
  SubscriberMock subscriber_mock;
  EXPECT_CALLS_ON_STATEMENT_AND_WAIT(subscriber_mock, callback(hasPoints()), sendActiveZone(0), 3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersForAllDefinedZoneTypes)
{
  SubscriberMock subscriber_mock;

  psen_scan_v2_standalone::util::Barrier safety_msg_received_barrier;
  psen_scan_v2_standalone::util::Barrier warn_msg_received_barrier;

  EXPECT_CALL(subscriber_mock, callback(matchesName("active zoneset safety1 min:-10.0 max:+10.0")))
      .WillOnce(OpenBarrier(&safety_msg_received_barrier));
  EXPECT_CALL(subscriber_mock, callback(matchesName("active zoneset warn1 min:-10.0 max:+10.0")))
      .WillOnce(OpenBarrier(&warn_msg_received_barrier));

  sendActiveZone(0);

  safety_msg_received_barrier.waitTillRelease(3s);
  warn_msg_received_barrier.waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersForNewActiveZoneWhenActiveZoneSwitches)
{
  SubscriberMock subscriber_mock;

  psen_scan_v2_standalone::util::Barrier safety_msg_received_barrier1;
  psen_scan_v2_standalone::util::Barrier warn_msg_received_barrier1;

  EXPECT_CALL(subscriber_mock, callback(matchesName("active zoneset safety1 min:-10.0 max:+10.0")))
      .WillOnce(OpenBarrier(&safety_msg_received_barrier1));
  EXPECT_CALL(subscriber_mock, callback(matchesName("active zoneset warn1 min:-10.0 max:+10.0")))
      .WillOnce(OpenBarrier(&warn_msg_received_barrier1));

  sendActiveZone(0);

  safety_msg_received_barrier1.waitTillRelease(3s);
  warn_msg_received_barrier1.waitTillRelease(3s);

  psen_scan_v2_standalone::util::Barrier safety_msg_received_barrier2;
  psen_scan_v2_standalone::util::Barrier warn_msg_received_barrier2;

  EXPECT_CALL(subscriber_mock, callback(matchesName("active zoneset safety1 min:+11.0 max:+50.0")))
      .WillOnce(OpenBarrier(&safety_msg_received_barrier2));
  EXPECT_CALL(subscriber_mock, callback(matchesName("active zoneset warn1 min:+11.0 max:+50.0")))
      .WillOnce(OpenBarrier(&warn_msg_received_barrier2));

  sendActiveZone(1);

  safety_msg_received_barrier2.waitTillRelease(3s);
  warn_msg_received_barrier2.waitTillRelease(3s);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_active_zoneset_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
