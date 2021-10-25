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
    subscriber_ = nh_.subscribe("/test_ns_laser_1/zonesets", 10, &SubscriberMock::callback, this);
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
      zoneset_config_ = readSingleMsgFromBagFile(bag_testfile, "/laser_1/zonesets");
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
  EXPECT_TRUE(TopicExists("/test_ns_laser_1/zonesets"));
}

TEST_F(ConfigServerNodeTest, shouldPublishLatchedOnZonesetTopic)
{
  SubscriberMock subscriber_mock;
  util::Barrier topic_received_barrier;

  EXPECT_CALL(subscriber_mock, callback(isLatched())).WillOnce(OpenBarrier(&topic_received_barrier));

  topic_received_barrier.waitTillRelease(3s);
}

TEST_F(ConfigServerNodeTest, shouldPublishMessageMatchingExpectedZoneSetsCount)
{
  SubscriberMock subscriber_mock;
  util::Barrier msg_received_barrier;

  EXPECT_CALL(subscriber_mock, callback(messageZoneSetCountEQ(expectedZoneSetConfig())))
      .WillOnce(OpenBarrier(&msg_received_barrier));

  msg_received_barrier.waitTillRelease(3s);
}

TEST_F(ConfigServerNodeTest, shouldPublishMessageMatchingExpectedZoneSetsSpeedLimits)
{
  SubscriberMock subscriber_mock;
  util::Barrier msg_received_barrier;

  EXPECT_CALL(subscriber_mock, callback(messageZoneSetsSpeedLimitsEQ(expectedZoneSetConfig())))
      .WillOnce(OpenBarrier(&msg_received_barrier));

  msg_received_barrier.waitTillRelease(3s);
}

TEST_F(ConfigServerNodeTest, shouldPublishMessageMatchingExpectedZoneSetsFrameId)
{
  SubscriberMock subscriber_mock;
  util::Barrier msg_received_barrier;

  EXPECT_CALL(subscriber_mock, callback(messageZoneSetsFrameIdEQ(expectedZoneSetConfig())))
      .WillOnce(OpenBarrier(&msg_received_barrier));

  msg_received_barrier.waitTillRelease(3s);
}

TEST_F(ConfigServerNodeTest, shouldPublishMessageMatchingExpectedPolygonPointCounts)
{
  SubscriberMock subscriber_mock;
  util::Barrier msg_received_barrier;

  EXPECT_CALL(subscriber_mock, callback(messageZoneSetsPolygonPointCountsEQ(expectedZoneSetConfig())))
      .WillOnce(OpenBarrier(&msg_received_barrier));

  msg_received_barrier.waitTillRelease(3s);
}

TEST_F(ConfigServerNodeTest, shouldPublishMessageMatchingExpectedPolygonPoints)
{
  // ToDo
  SUCCEED();
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
