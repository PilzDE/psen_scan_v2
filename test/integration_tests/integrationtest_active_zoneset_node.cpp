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

#include "psen_scan_v2/active_zoneset_node.h"

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

MATCHER_P(matchesName, expectedName, "")
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

  bool isConnected(const ros::Duration& timeout = ros::Duration(10.0)) const
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
  EXPECT_CALL_X_TIMES_RUN_STATEMENT_AND_WAIT(marker_sub_mock_, callback(isTriangleList()), 2, sendActiveZone(0);, 3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkerWithPoints)
{
  EXPECT_CALL_X_TIMES_RUN_STATEMENT_AND_WAIT(marker_sub_mock_, callback(hasPoints()), 2, sendActiveZone(0);, 3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersForAllDefinedZoneTypes)
{
#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10.0 max:+10.0"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10.0 max:+10.0"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
#else
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10 max:+10"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10 max:+10"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
#endif
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersForNewActiveZoneWhenActiveZoneSwitches)
{
#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10.0 max:+10.0"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10.0 max:+10.0"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
  EXPECT_4_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10.0 max:+10.0"), hasDeleteAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10.0 max:+10.0"), hasDeleteAction())),
      callback(AllOf(matchesName("active zoneset safety1 min:+11.0 max:+50.0"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:+11.0 max:+50.0"), hasAddAction())),
      sendActiveZone(1);
      , 3s);
#else
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10 max:+10"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10 max:+10"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
  EXPECT_4_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10 max:+10"), hasDeleteAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10 max:+10"), hasDeleteAction())),
      callback(AllOf(matchesName("active zoneset safety1 min:+11 max:+50"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:+11 max:+50"), hasAddAction())),
      sendActiveZone(1);
      , 3s);
#endif
}

TEST_F(ActiveZonesetNodeTest, shouldNotPublishDeleteMarkersForSameActiveZone)
{
#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10.0 max:+10.0"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10.0 max:+10.0"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10.0 max:+10.0"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10.0 max:+10.0"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
#else
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10 max:+10"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10 max:+10"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10 max:+10"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10 max:+10"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
#endif
}

TEST_F(ActiveZonesetNodeTest, shouldPublishDeleteMarkersOnInvalidActiveZone)
{
#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10.0 max:+10.0"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10.0 max:+10.0"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10.0 max:+10.0"), hasDeleteAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10.0 max:+10.0"), hasDeleteAction())),
      sendActiveZone(5);
      , 3s);
#else
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10 max:+10"), hasAddAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10 max:+10"), hasAddAction())),
      sendActiveZone(0);
      , 3s);
  EXPECT_2_CALLS_RUN_STATEMENT_AND_WAIT(
      marker_sub_mock_,
      callback(AllOf(matchesName("active zoneset safety1 min:-10 max:+10"), hasDeleteAction())),
      callback(AllOf(matchesName("active zoneset warn1 min:-10 max:+10"), hasDeleteAction())),
      sendActiveZone(5);
      , 3s);
#endif
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
