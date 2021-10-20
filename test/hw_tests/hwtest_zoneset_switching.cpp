// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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
#include <future>

#include <string>

#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>

#include "psen_scan_v2_standalone/util/expectations.h"

namespace psen_scan_v2_test
{
static constexpr int32_t WAIT_FOR_MESSAGE_TIMEOUT_S{ 5 };
static constexpr uint8_t ZONE_ZERO_CMD{ 4 };
static constexpr uint8_t ZONE_ONE_CMD{ 12 };

class ScanComparisonTests : public ::testing::Test
{
public:
  void SetUp() override  // Omit using SetUpTestSuite() for googletest below v1.11.0, see
                         // https://github.com/google/googletest/issues/247
  {
    ros::NodeHandle pnh{ "~" };
    pub_relay_cmd_ = pnh.advertise<std_msgs::Byte>("/relay_cmd", 1);
  }

  void TearDown() override
  {
    setScannerZoneSet(ZONE_ZERO_CMD);
  }

protected:
  void setScannerZoneSet(uint8_t cmd);

protected:
  ros::Publisher pub_relay_cmd_;
};

inline void ScanComparisonTests::setScannerZoneSet(uint8_t cmd)
{
  std_msgs::Byte command;
  command.data = cmd;
  pub_relay_cmd_.publish(command);
  ros::topic::waitForMessage<std_msgs::Byte>("/relay_cmd", ros::Duration(WAIT_FOR_MESSAGE_TIMEOUT_S, 0));
}

static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 3 };
static constexpr std::chrono::milliseconds NOT_SET_TIMEOUT{ 5 };

TEST_F(ScanComparisonTests, simpleCompare)
{
  ros::NodeHandle nh;
  std::promise<void> received_active_zone_one_promise;
  auto received_active_zone_one = received_active_zone_one_promise.get_future();
  std::promise<void> received_active_zone_two_promise;
  auto received_active_zone_two = received_active_zone_two_promise.get_future();

  boost::function<void(const std_msgs::UInt8&)> callback = [&](const std_msgs::UInt8& msg) {
    if (msg.data == 0 && received_active_zone_one.wait_for(NOT_SET_TIMEOUT) == std::future_status::timeout)
    {
      received_active_zone_one_promise.set_value();
    }
    else if (msg.data == 1 && received_active_zone_two.wait_for(NOT_SET_TIMEOUT) == std::future_status::timeout)
    {
      received_active_zone_two_promise.set_value();
    }
  };
  auto zone_subscriber = nh.subscribe<std_msgs::UInt8>("/laser_1/active_zoneset", 10, callback);

  setScannerZoneSet(ZONE_ZERO_CMD);
  EXPECT_FUTURE_IS_READY(received_active_zone_one, DEFAULT_TIMEOUT);
  setScannerZoneSet(ZONE_ONE_CMD);
  EXPECT_FUTURE_IS_READY(received_active_zone_two, DEFAULT_TIMEOUT);
}

}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "active_zoneset_test");

  // Needed since we use a subscriber
  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  return RUN_ALL_TESTS();
}
