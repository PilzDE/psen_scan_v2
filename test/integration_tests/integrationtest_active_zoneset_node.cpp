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
#include <vector>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

// Needed only for FMT_VERSION define but version 4.0.0 used on Ubuntu 18 (melodic) does not yet use a core.h
#include <fmt/format.h>

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
  *result_listener << "arg.points length is: " << arg->points.size();
  return !arg->points.empty();
}

MATCHER(isTriangleList, "")
{
  *result_listener << "arg.type is: " << arg->type << "but should be: " << visualization_msgs::Marker::TRIANGLE_LIST;
  return arg->type == visualization_msgs::Marker::TRIANGLE_LIST;
}

MATCHER_P(hasNS, expectedName, "")
{
  *result_listener << "arg.ns is: " << arg->ns << " but should be: " << expectedName;
  return arg->ns == expectedName;
}

MATCHER(hasDeleteAction, "")
{
  *result_listener << "arg.action is: " << arg->action << " but should be: " << visualization_msgs::Marker::DELETE;
  return arg->action == visualization_msgs::Marker::DELETE;
}

MATCHER(hasAddAction, "")
{
  *result_listener << "arg.action is: " << arg->action << " but should be: " << visualization_msgs::Marker::ADD;
  return arg->action == visualization_msgs::Marker::ADD;
}

class MarkerSubscriberMock
{
public:
  MarkerSubscriberMock()
  {
    subscriber_ = nh_.subscribe("/test_ns_laser_1/active_zoneset_marker", 10, &MarkerSubscriberMock::callback, this);
  }

  bool isConnected(const ros::Duration& timeout = ros::Duration(3.0)) const
  {
    const auto start_time = ros::Time::now();
    while (ros::ok())
    {
      if (subscriber_.getNumPublishers() == 1)
      {
        return true;
      }
      if ((ros::Time::now() - start_time) > timeout)
      {
        return false;
      }
      ros::Duration(0.1).sleep();
    }
    return false;
  }

  MOCK_METHOD1(callback, void(const visualization_msgs::MarkerConstPtr& event));

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};

#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
static const std::string SAFETY_NS_ZONE_1{ "active zoneset safety1 min:-10.0 max:+10.0" };
static const std::string WARN_NS_ZONE_1{ "active zoneset warn1 min:-10.0 max:+10.0" };
static const std::string SAFETY_NS_ZONE_2{ "active zoneset safety1 min:+11.0 max:+50.0" };
static const std::string WARN_NS_ZONE_2 = { "active zoneset warn1 min:+11.0 max:+50.0" };
#else
static const std::string SAFETY_NS_ZONE_1{ "active zoneset safety1 min:-10 max:+10" };
static const std::string WARN_NS_ZONE_1{ "active zoneset warn1 min:-10 max:+10" };
static const std::string SAFETY_NS_ZONE_2{ "active zoneset safety1 min:+11 max:+50" };
static const std::string WARN_NS_ZONE_2{ "active zoneset warn1 min:+11 max:+50" };
#endif

class ActiveZonesetNodeTest : public testing::Test
{
public:
  void SetUp() override
  {
    ros::NodeHandle nh;
    pub_active_ = nh.advertise<std_msgs::UInt8>("/test_ns_laser_1/active_zoneset", 1, true);
    ASSERT_TRUE(marker_sub_mock_.isConnected());
  }
  void sendActiveZone(uint8_t zone);

public:
  MarkerSubscriberMock marker_sub_mock_;
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
  auto barrier = EXPECT_N_ASYNC_CALLS(marker_sub_mock_, callback(isTriangleList()), 2);
  sendActiveZone(0);
  barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkerWithPoints)
{
  auto barrier = EXPECT_N_ASYNC_CALLS(marker_sub_mock_, callback(hasPoints()), 2);
  sendActiveZone(0);
  barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersForAllDefinedZoneTypes)
{
  auto s_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(SAFETY_NS_ZONE_1), hasAddAction())));
  auto w_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(WARN_NS_ZONE_1), hasAddAction())));
  sendActiveZone(0);
  s_barrier->waitTillRelease(3s);
  w_barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersForNewActiveZoneWhenActiveZoneSwitches)
{
  auto s1_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(SAFETY_NS_ZONE_1), hasAddAction())));
  auto w1_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(WARN_NS_ZONE_1), hasAddAction())));
  sendActiveZone(0);
  s1_barrier->waitTillRelease(3s);
  w1_barrier->waitTillRelease(3s);

  auto d_barrier = EXPECT_N_ASYNC_CALLS(marker_sub_mock_, callback(hasDeleteAction()), 2);
  auto s2_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(SAFETY_NS_ZONE_2), hasAddAction())));
  auto w2_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(WARN_NS_ZONE_2), hasAddAction())));
  sendActiveZone(1);
  d_barrier->waitTillRelease(3s);
  s2_barrier->waitTillRelease(3s);
  w2_barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldNotPublishDeleteMarkersForSameActiveZone)
{
  auto s_barrier = EXPECT_N_ASYNC_CALLS(marker_sub_mock_, callback(AllOf(hasNS(SAFETY_NS_ZONE_1), hasAddAction())), 2);
  auto w_barrier = EXPECT_N_ASYNC_CALLS(marker_sub_mock_, callback(AllOf(hasNS(WARN_NS_ZONE_1), hasAddAction())), 2);
  sendActiveZone(0);
  ros::Duration(0.5).sleep();
  sendActiveZone(0);
  s_barrier->waitTillRelease(3s);
  w_barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishDeleteMarkersOnInvalidActiveZone)
{
  auto s1_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(SAFETY_NS_ZONE_1), hasAddAction())));
  auto w1_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(WARN_NS_ZONE_1), hasAddAction())));
  sendActiveZone(0);
  s1_barrier->waitTillRelease(3s);
  w1_barrier->waitTillRelease(3s);

  auto s1d_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(SAFETY_NS_ZONE_1), hasDeleteAction())));
  auto w1d_barrier = EXPECT_ASYNC_CALL(marker_sub_mock_, callback(AllOf(hasNS(WARN_NS_ZONE_1), hasDeleteAction())));
  sendActiveZone(5);
  s1d_barrier->waitTillRelease(3s);
  w1d_barrier->waitTillRelease(3s);
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
