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
#include <memory>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

// Needed only for FMT_VERSION define but version 4.0.0 used on Ubuntu 18 (melodic) does not yet use a core.h
#include <fmt/format.h>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/UInt8.h>

#include "psen_scan_v2/ros_integrationtest_helper.h"
#include "psen_scan_v2/subscriber_mock.h"

#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/gmock_expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

namespace psen_scan_v2_test
{
using namespace std::chrono_literals;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_standalone_test;

using ::testing::Contains;

MATCHER(hasPoints, "")
{
  return !arg.points.empty();
}

MATCHER(hasTriangleList, "")
{
  *result_listener << "arg.type is: " << arg.type << " but should be: " << visualization_msgs::Marker::TRIANGLE_LIST;
  return arg.type == visualization_msgs::Marker::TRIANGLE_LIST;
}

MATCHER_P(hasNS, expectedNS, "")
{
  *result_listener << "arg.ns is: " << arg.ns << " but should be: " << expectedNS;
  return arg.ns == expectedNS;
}

MATCHER(hasDeleteAction, "")
{
  *result_listener << "arg.action is: " << arg.action << " but should be: " << visualization_msgs::Marker::DELETE;
  return arg.action == visualization_msgs::Marker::DELETE;
  // return true;
}

MATCHER(hasAddAction, "")
{
  *result_listener << "arg.action is: " << arg.action << " but should be: " << visualization_msgs::Marker::ADD;
  return arg.action == visualization_msgs::Marker::ADD;
  // return true;
}

static constexpr int QUEUE_SIZE{ 10 };

#if (FMT_VERSION >= 60000 && FMT_VERSION < 70100)
static const std::string SAFETY_NS_ZONE_1{ "active zoneset safety1 min:-10.0 max:+10.0" };
static const std::string WARN_NS_ZONE_1{ "active zoneset warn1 min:-10.0 max:+10.0" };
static const std::string SAFETY_NS_ZONE_2{ "active zoneset safety1 min:+11.0 max:+50.0" };
#else
static const std::string SAFETY_NS_ZONE_1{ "active zoneset safety1 min:-10 max:+10" };
static const std::string WARN_NS_ZONE_1{ "active zoneset warn1 min:-10 max:+10" };
static const std::string SAFETY_NS_ZONE_2{ "active zoneset safety1 min:+11 max:+50" };
#endif

TEST(ActiveZonesetNodeTopicTest, shouldAdvertiseZonesetMarkersTopic)
{
  EXPECT_TRUE(TopicExists("/test_ns_laser_1/active_zoneset_markers"));
}

class ActiveZonesetNodeTest : public testing::Test
{
public:
  void SetUp() override;
  void sendActiveZone(uint8_t zone);
  ::testing::AssertionResult resetActiveZoneNode();
  ::testing::AssertionResult switchToInvalidActiveZoneAfterSetup();

public:
  std::unique_ptr<SubscriberMock<visualization_msgs::MarkerArray>> marker_sub_mock_;
  ros::NodeHandle nh_;
  ros::Publisher pub_active_;
};

void ActiveZonesetNodeTest::SetUp()
{
  pub_active_ = nh_.advertise<std_msgs::UInt8>("/test_ns_laser_1/active_zoneset", 10, true);

  // reset active zone to create clear testing conditions
  ASSERT_TRUE(resetActiveZoneNode());

  // initialize here to avoid traffic of above reset
  marker_sub_mock_.reset(new SubscriberMock<visualization_msgs::MarkerArray>{
      nh_, "/test_ns_laser_1/active_zoneset_markers", QUEUE_SIZE });
  ASSERT_TRUE(isConnected(*marker_sub_mock_));
}

const std::string ACTIVE_ZONESET_MARKER_TOPICNAME{ "/test_ns_laser_1/active_zoneset_markers" };

::testing::AssertionResult ActiveZonesetNodeTest::switchToInvalidActiveZoneAfterSetup()
{
  SubscriberMock<visualization_msgs::MarkerArray> invalid_marker_mock(nh_, ACTIVE_ZONESET_MARKER_TOPICNAME, QUEUE_SIZE);
  if (!isConnected(invalid_marker_mock))
  {
    return ::testing::AssertionFailure() << "Could not connect with subscriber on marker topic.";
  }

  util::Barrier invalid_marker_barrier;
  ON_CALL(invalid_marker_mock, callback(Field(&visualization_msgs::MarkerArray::markers, Contains(hasDeleteAction()))))
      .WillByDefault(OpenBarrier(&invalid_marker_barrier));

  sendActiveZone(5);
  if (!invalid_marker_barrier.waitTillRelease(3s))
  {
    return ::testing::AssertionFailure() << "Failed to receive markers for deletion.";
  }
  return ::testing::AssertionSuccess();
}

void ActiveZonesetNodeTest::sendActiveZone(uint8_t zone)
{
  std_msgs::UInt8 active_zone_msg;
  active_zone_msg.data = zone;
  pub_active_.publish(active_zone_msg);
}

::testing::AssertionResult ActiveZonesetNodeTest::resetActiveZoneNode()
{
  SubscriberMock<visualization_msgs::MarkerArray> reset_marker_mock(nh_, ACTIVE_ZONESET_MARKER_TOPICNAME, QUEUE_SIZE);

  if (!isConnected(reset_marker_mock))
  {
    return ::testing::AssertionFailure() << "Could not connect with subscriber on marker topic.";
  }

  auto reset_marker_barrier = EXPECT_ASYNC_CALL(
      reset_marker_mock, callback(Field(&visualization_msgs::MarkerArray::markers, Contains(hasAddAction()))));

  sendActiveZone(0);
  if (!reset_marker_barrier->waitTillRelease(3s))
  {
    return ::testing::AssertionFailure() << "Failed to receive 1 markerarray to add for active zone 0.";
  }
  return ::testing::AssertionSuccess();
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersWithCorrectType)
{
  auto barrier = EXPECT_N_ASYNC_CALLS(
      *marker_sub_mock_, callback(Field(&visualization_msgs::MarkerArray::markers, Contains(hasTriangleList()))), 1);
  sendActiveZone(0);
  barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersWithPoints)
{
  auto barrier = EXPECT_N_ASYNC_CALLS(
      *marker_sub_mock_, callback(Field(&visualization_msgs::MarkerArray::markers, Contains(hasPoints()))), 1);
  sendActiveZone(0);
  barrier->waitTillRelease(3s);
}

using ::testing::AllOf;

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersForAllDefinedZoneTypes)
{
  auto barrier = EXPECT_ASYNC_CALL(*marker_sub_mock_,
                                   callback(Field(&visualization_msgs::MarkerArray::markers,
                                                  AllOf(SizeIs(2),
                                                        Contains(AllOf(hasNS(SAFETY_NS_ZONE_1), hasAddAction())),
                                                        Contains(AllOf(hasNS(WARN_NS_ZONE_1), hasAddAction()))))));
  sendActiveZone(0);
  barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersForNewActiveZoneWhenActiveZoneSwitches)
{
  auto barrier = EXPECT_ASYNC_CALL(*marker_sub_mock_,
                                   callback(Field(&visualization_msgs::MarkerArray::markers,
                                                  AllOf(SizeIs(3),
                                                        Contains(AllOf(hasNS(SAFETY_NS_ZONE_1), hasDeleteAction())),
                                                        Contains(AllOf(hasNS(WARN_NS_ZONE_1), hasDeleteAction())),
                                                        Contains(AllOf(hasNS(SAFETY_NS_ZONE_2), hasAddAction()))))));
  sendActiveZone(1);
  barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldNotPublishDeleteMarkersForSameActiveZone)
{
  EXPECT_CALL(*marker_sub_mock_,
              callback(Field(&visualization_msgs::MarkerArray::markers,
                             AnyOf(Contains(AllOf(hasNS(SAFETY_NS_ZONE_1), hasDeleteAction())),
                                   Contains(AllOf(hasNS(WARN_NS_ZONE_1), hasDeleteAction()))))))
      .Times(0);
  auto barrier = EXPECT_ASYNC_CALL(*marker_sub_mock_,
                                   callback(Field(&visualization_msgs::MarkerArray::markers,
                                                  AllOf(Contains(AllOf(hasNS(SAFETY_NS_ZONE_1), hasAddAction())),
                                                        Contains(AllOf(hasNS(WARN_NS_ZONE_1), hasAddAction()))))));
  sendActiveZone(0);
  barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishDeleteMarkersOnInvalidActiveZone)
{
  auto barrier = EXPECT_ASYNC_CALL(*marker_sub_mock_,
                                   callback(Field(&visualization_msgs::MarkerArray::markers,
                                                  AllOf(Contains(AllOf(hasNS(SAFETY_NS_ZONE_1), hasDeleteAction())),
                                                        Contains(AllOf(hasNS(WARN_NS_ZONE_1), hasDeleteAction()))))));
  sendActiveZone(5);
  barrier->waitTillRelease(3s);
}

TEST_F(ActiveZonesetNodeTest, shouldPublishMarkersForNewActiveZoneAfterSwitchFromInvalidActiveZone)
{
  ASSERT_TRUE(switchToInvalidActiveZoneAfterSetup());

  auto barrier = EXPECT_ASYNC_CALL(*marker_sub_mock_,
                                   callback(Field(&visualization_msgs::MarkerArray::markers,
                                                  AllOf(Contains(AllOf(hasNS(SAFETY_NS_ZONE_1), hasAddAction())),
                                                        Contains(AllOf(hasNS(WARN_NS_ZONE_1), hasAddAction()))))));
  sendActiveZone(0);
  barrier->waitTillRelease(3s);
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
