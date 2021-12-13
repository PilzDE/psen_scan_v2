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

#include <thread>
#include <chrono>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "psen_scan_v2/config_server_node.h"

#include "psen_scan_v2/ZoneSet.h"
#include "psen_scan_v2/ZoneSetConfiguration.h"
#include "psen_scan_v2/zoneset_msg_builder.h"

#include "psen_scan_v2/ros_integrationtest_helper.h"
#include "psen_scan_v2/subscriber_mock.h"

#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_standalone_test;
using namespace std::chrono_literals;

static const std::string FRAME_ID_PARAM_NAME{ "frame_id" };
static const std::string BAG_TESTFILE_PARAM_NAME{ "bag_testfile" };

namespace psen_scan_v2_test
{
using ::testing::ExplainMatchResult;
using ::testing::SizeIs;

MATCHER_P2(namedEQ, value, name, "")
{
  *result_listener << name << " " << arg << " " << (value == arg ? "does" : "doesn't") << " match";
  return value == arg;
}

MATCHER_P2(namedLessVerboseEQ, value, name, "")
{
  *result_listener << name << " " << (value == arg ? "does" : "doesn't") << " match";
  return value == arg;
}

MATCHER_P2(namedZoneSetPolygonEQ, polygon, name, "")
{
  *result_listener << name << " with points vector, ";
  return ExplainMatchResult(SizeIs(polygon.points.size()), arg.points, result_listener) &&
         ExplainMatchResult(namedLessVerboseEQ(polygon.points, ", whose data"), arg.points, result_listener);
}

MATCHER_P(zoneSetEQ, zoneset, "")
{
  *result_listener << "whose fields";
  return ExplainMatchResult(namedEQ(zoneset.header.frame_id, "\n\tframe_id"), arg.header.frame_id, result_listener) &&
         ExplainMatchResult(namedEQ(zoneset.speed_lower, "\n\tspeed_lower"), arg.speed_lower, result_listener) &&
         ExplainMatchResult(namedEQ(zoneset.speed_upper, "\n\tspeed_upper"), arg.speed_upper, result_listener) &&
         ExplainMatchResult(namedZoneSetPolygonEQ(zoneset.safety1, "\n\tsafety1"), arg.safety1, result_listener) &&
         ExplainMatchResult(namedZoneSetPolygonEQ(zoneset.safety2, "\n\tsafety2"), arg.safety2, result_listener) &&
         ExplainMatchResult(namedZoneSetPolygonEQ(zoneset.safety3, "\n\tsafety3"), arg.safety3, result_listener) &&
         ExplainMatchResult(namedZoneSetPolygonEQ(zoneset.warn1, "\n\twarn1"), arg.warn1, result_listener) &&
         ExplainMatchResult(namedZoneSetPolygonEQ(zoneset.warn2, "\n\twarn2"), arg.warn2, result_listener) &&
         ExplainMatchResult(namedZoneSetPolygonEQ(zoneset.muting1, "\n\tmuting1"), arg.muting1, result_listener) &&
         ExplainMatchResult(namedZoneSetPolygonEQ(zoneset.muting2, "\n\tmuting2"), arg.muting2, result_listener);
}

MATCHER_P(zoneSetVecEQ, zoneset_vec, "")
{
  unsigned int i = 0;
  return ExplainMatchResult(SizeIs(zoneset_vec.size()), arg, result_listener) &&
         std::equal(
             zoneset_vec.begin(), zoneset_vec.end(), arg.begin(), arg.end(), [&](const auto& zs1, const auto& zs2) {
               *result_listener << "\nwith zoneset" << i++ << " ";
               return ExplainMatchResult(zoneSetEQ(zs1), zs2, result_listener);
             });
}

MATCHER_P(msgZoneSetConfigEQ, expected_msg, "")
{
  auto actual_msg = arg.getMessage();
  *result_listener << "with zonesets vector, ";
  return ExplainMatchResult(zoneSetVecEQ(expected_msg.zonesets), actual_msg->zonesets, result_listener);
}

ZoneSetConfiguration readSingleMsgFromBagFile(const std::string& filepath, const std::string& zone_sets_topic)
{
  rosbag::Bag bag;
  bag.open(filepath, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery(zone_sets_topic));

  const auto msg = view.begin()->instantiate<ZoneSetConfiguration>();

  bag.close();
  return *msg;
}
class ConfigServerNodeTest : public testing::Test
{
public:
  void SetUp() override
  {
    ros::NodeHandle nh{ "~" };
    ASSERT_TRUE(nh.getParam(FRAME_ID_PARAM_NAME, frame_id_))
        << "Parameter " << FRAME_ID_PARAM_NAME << " does not exist.";

    std::string bag_testfile;
    ASSERT_TRUE(nh.getParam(BAG_TESTFILE_PARAM_NAME, bag_testfile))
        << "Parameter " << BAG_TESTFILE_PARAM_NAME << " does not exist.";

    try
    {
      zoneset_config_ = readSingleMsgFromBagFile(bag_testfile, "/laser_1/zoneconfiguration");
    }
    catch (const rosbag::BagIOException& e)
    {
      FAIL() << "File " << bag_testfile
             << " could not be opened. Make sure the file exists and that you have sufficient rights to open it.";
    }
    catch (const rosbag::BagException& e)
    {
      FAIL() << "There was an error opening " << bag_testfile;
    }
  }

  ZoneSetConfiguration expectedZoneSetConfig() const
  {
    return zoneset_config_;
  }

private:
  std::string frame_id_;
  ZoneSetConfiguration zoneset_config_;
};

const std::string ZONE_CONFIGURATION_TOPICNAME{ "/test_ns_laser_1/zoneconfiguration" };

TEST_F(ConfigServerNodeTest, shouldAdvertiseZonesetTopic)
{
  // Set param on server
  EXPECT_TRUE(TopicExists(ZONE_CONFIGURATION_TOPICNAME));
}

static constexpr int QUEUE_SIZE{ 10 };

TEST_F(ConfigServerNodeTest, shouldPublishLatchedOnZonesetTopic)
{
  ros::NodeHandle nh;
  SubscriberMock<ros::MessageEvent<ZoneSetConfiguration const>> subscriber_mock(
      nh, ZONE_CONFIGURATION_TOPICNAME, QUEUE_SIZE);
  util::Barrier topic_received_barrier;

  EXPECT_CALL(subscriber_mock, callback(isLatched())).WillOnce(OpenBarrier(&topic_received_barrier));

  topic_received_barrier.waitTillRelease(3s);
}

TEST_F(ConfigServerNodeTest, shouldPublishMessageMatchingExpectedZoneSetConfig)
{
  ros::NodeHandle nh;
  SubscriberMock<ros::MessageEvent<ZoneSetConfiguration const>> subscriber_mock(
      nh, ZONE_CONFIGURATION_TOPICNAME, QUEUE_SIZE);
  util::Barrier msg_received_barrier;

  EXPECT_CALL(subscriber_mock, callback(msgZoneSetConfigEQ(expectedZoneSetConfig())))
      .WillOnce(OpenBarrier(&msg_received_barrier));

  msg_received_barrier.waitTillRelease(3s);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_config_server_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
