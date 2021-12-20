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

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <string>
#include <functional>

#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>

#include "psen_scan_v2_standalone/util/gtest_expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"
#include "psen_scan_v2_standalone/util/async_barrier.h"

#include "psen_scan_v2/ros_integrationtest_helper.h"
#include "psen_scan_v2/subscriber_mock.h"

namespace psen_scan_v2_test
{
namespace util = psen_scan_v2_standalone::util;
using psen_scan_v2_standalone_test::OpenBarrier;
using ::testing::AnyNumber;

static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 5 };
static constexpr uint8_t ZONE_ZERO_CMD{ 4 };
static constexpr uint8_t ZONE_ONE_CMD{ 12 };

std_msgs::UInt8 createActiveZonesetMsg(const uint8_t active_zoneset)
{
  std_msgs::UInt8 zone;
  zone.data = active_zoneset;
  return zone;
}

static constexpr int QUEUE_SIZE{ 1 };
const std::string ACTIVE_ZONESET_TOPICNAME{ "/laser_1/active_zoneset" };
class ActiveZonesetSwitchTests : public ::testing::Test
{
public:
  void SetUp() override
  {
    pub_relay_cmd_ = nh_.advertise<std_msgs::Byte>("/relay_cmd", 1);
    setScannerZoneSet(ZONE_ZERO_CMD);
    util::Barrier zone_zero_barrier;
    SubscriberMock<std_msgs::UInt8> sm{ nh_, ACTIVE_ZONESET_TOPICNAME, QUEUE_SIZE };
    EXPECT_CALL(sm, callback(::testing::_)).Times(AnyNumber());
    EXPECT_CALL(sm, callback(createActiveZonesetMsg(0))).Times(AnyNumber()).WillOnce(OpenBarrier(&zone_zero_barrier));
    zone_zero_barrier.waitTillRelease(DEFAULT_TIMEOUT);
  }

  void TearDown() override
  {
    setScannerZoneSet(ZONE_ZERO_CMD);
  }

protected:
  void setScannerZoneSet(uint8_t cmd);

protected:
  ros::Publisher pub_relay_cmd_;
  ros::NodeHandle nh_;
};

inline void ActiveZonesetSwitchTests::setScannerZoneSet(uint8_t cmd)
{
  std_msgs::Byte command;
  command.data = cmd;
  pub_relay_cmd_.publish(command);
}

TEST_F(ActiveZonesetSwitchTests, shouldPublishChangedZonesetIfIOChanges)
{
  util::Barrier zone_zero_barrier;
  util::Barrier zone_one_barrier;

  SubscriberMock<std_msgs::UInt8> sm{ nh_, ACTIVE_ZONESET_TOPICNAME, QUEUE_SIZE };
  {
    ::testing::InSequence s;
    EXPECT_CALL(sm, callback(createActiveZonesetMsg(0))).Times(AnyNumber()).WillOnce(OpenBarrier(&zone_zero_barrier));
    EXPECT_CALL(sm, callback(createActiveZonesetMsg(1))).Times(AnyNumber()).WillOnce(OpenBarrier(&zone_one_barrier));
  }

  setScannerZoneSet(ZONE_ZERO_CMD);
  EXPECT_BARRIER_OPENS(zone_zero_barrier, DEFAULT_TIMEOUT);
  setScannerZoneSet(ZONE_ONE_CMD);
  EXPECT_BARRIER_OPENS(zone_one_barrier, DEFAULT_TIMEOUT);
}

}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "active_zoneset_test");
  ros::NodeHandle pn;

  // Needed since we use a subscriber
  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  return RUN_ALL_TESTS();
}
