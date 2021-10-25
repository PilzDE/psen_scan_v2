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
#include "psen_scan_v2/config_server_node.h"

#include "psen_scan_v2/ZoneSet.h"
#include "psen_scan_v2/ZoneSetConfiguration.h"

#include "psen_scan_v2/ros_integrationtest_helper.h"

#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_standalone_test;
using namespace std::chrono_literals;

static const std::string FRAME_ID_PARAM_NAME{ "frame_id" };

namespace psen_scan_v2_test
{
class SubscriberMock
{
public:
  SubscriberMock()
  {
    subscriber_ = nh_.subscribe("/test_ns_laser_1/zonesets", 10, &SubscriberMock::callback, this);
  }

  // MOCK_METHOD1(callback, void(const ZoneSetConfiguration& msg));
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
  }

  ZoneSetConfiguration expectedZoneSetConfig() const
  {
    ZoneSetConfiguration zoneset_config;
    ZoneSet zoneset0;
    zoneset0.header.frame_id = frame_id_;
    zoneset0.speed_lower = -10;
    zoneset0.speed_upper = 10;
    zoneset0.safety1.points.resize(550);
    zoneset0.warn1.points.resize(550);
    ZoneSet zoneset1;
    zoneset1.header.frame_id = frame_id_;
    zoneset1.speed_lower = 11;
    zoneset1.speed_upper = 50;
    zoneset1.safety1.points.resize(550);
    zoneset1.warn1.points.resize(550);
    zoneset_config.zonesets.push_back(zoneset0);
    zoneset_config.zonesets.push_back(zoneset1);

    return zoneset_config;
  }

private:
  std::string frame_id_;
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

TEST_F(ConfigServerNodeTest, shouldPublishMessageMatchingExpectedZoneSetCount)
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
