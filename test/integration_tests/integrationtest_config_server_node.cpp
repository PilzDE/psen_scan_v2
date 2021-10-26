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

#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "psen_scan_v2/config_server_node.h"

#include "psen_scan_v2/ZoneSet.h"
#include "psen_scan_v2/ZoneSetConfiguration.h"
#include "psen_scan_v2/zoneset_msg_builder.h"

#include "psen_scan_v2/ros_integrationtest_helper.h"

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
using ::testing::Field;
using ::testing::SizeIs;

MATCHER_P(lessVerboseContainerEQ, expected, "")
{
  *result_listener << "whose elements " << (expected == arg ? "do" : "don't") << " match";
  return expected == arg;
}

MATCHER_P(zoneSetPolygonEQ, expected_polygon, "")
{
  *result_listener << "with points vector, ";
  return ExplainMatchResult(SizeIs(expected_polygon.points.size()), arg.points, result_listener) &&
         ExplainMatchResult(
             lessVerboseContainerEQ(expected_polygon.points), arg.points, &(*result_listener << " and "));
}

MATCHER_P(zoneSetEQ, expected_zoneset, "")
{
  return ExplainMatchResult(Field("header.frame_id", &std_msgs::Header::frame_id, expected_zoneset.header.frame_id),
                            arg.header,
                            &(*result_listener << "\t")) &&
         ExplainMatchResult(Field("speed_lower", &ZoneSet::speed_lower, expected_zoneset.speed_lower),
                            arg,
                            &(*result_listener << "\n\t")) &&
         ExplainMatchResult(Field("speed_upper", &ZoneSet::speed_upper, expected_zoneset.speed_upper),
                            arg,
                            &(*result_listener << "\n\t")) &&
         ExplainMatchResult(
             zoneSetPolygonEQ(expected_zoneset.safety1), arg.safety1, &(*result_listener << "\n\tfield safety1 ")) &&
         ExplainMatchResult(
             zoneSetPolygonEQ(expected_zoneset.safety2), arg.safety2, &(*result_listener << "\n\tfield safety2 ")) &&
         ExplainMatchResult(
             zoneSetPolygonEQ(expected_zoneset.safety3), arg.safety3, &(*result_listener << "\n\tfield safety3 ")) &&
         ExplainMatchResult(
             zoneSetPolygonEQ(expected_zoneset.warn1), arg.warn1, &(*result_listener << "\n\tfield warn1 ")) &&
         ExplainMatchResult(
             zoneSetPolygonEQ(expected_zoneset.warn2), arg.warn2, &(*result_listener << "\n\tfield warn2 ")) &&
         ExplainMatchResult(
             zoneSetPolygonEQ(expected_zoneset.muting1), arg.muting1, &(*result_listener << "\n\tfield muting1 ")) &&
         ExplainMatchResult(
             zoneSetPolygonEQ(expected_zoneset.muting2), arg.muting2, &(*result_listener << "\n\tfield muting2 "));
}

MATCHER_P(zoneSetVecEQ, expected_zonesets, "")
{
  unsigned int i = 0;
  return ExplainMatchResult(SizeIs(expected_zonesets.size()), arg, result_listener) &&
         std::equal(expected_zonesets.begin(),
                    expected_zonesets.end(),
                    arg.begin(),
                    arg.end(),
                    [&](const auto& zs1, const auto& zs2) {
                      *result_listener << "\nwith zoneset" << i++ << "\n";
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

class SubscriberMock
{
public:
  SubscriberMock()
  {
    subscriber_ = nh_.subscribe("/test_ns_laser_1/zoneconfiguration", 10, &SubscriberMock::callback, this);
  }

  MOCK_METHOD1(callback, void(const ros::MessageEvent<ZoneSetConfiguration const>& event));

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};

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

TEST_F(ConfigServerNodeTest, shouldAdvertiseZonesetTopic)
{
  // Set param on server
  EXPECT_TRUE(TopicExists("/test_ns_laser_1/zoneconfiguration"));
}

TEST_F(ConfigServerNodeTest, shouldPublishLatchedOnZonesetTopic)
{
  SubscriberMock subscriber_mock;
  util::Barrier topic_received_barrier;

  EXPECT_CALL(subscriber_mock, callback(isLatched())).WillOnce(OpenBarrier(&topic_received_barrier));

  topic_received_barrier.waitTillRelease(3s);
}

TEST_F(ConfigServerNodeTest, shouldPublishMessageMatchingExpectedZoneSetConfig)
{
  SubscriberMock subscriber_mock;
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
