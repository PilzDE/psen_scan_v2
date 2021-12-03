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
#include <map>
#include <ostream>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <std_msgs/Byte.h>
#include <std_msgs/UInt8.h>

#include "psen_scan_v2/IOState.h"

#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/gmock_expectations.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"

namespace psen_scan_v2
{
// Avoids too much output with full io pin data
void PrintTo(const IOState& io_state, std::ostream* os)
{
  *os << "IOState(...)";
}
}  // namespace psen_scan_v2

using namespace ::testing;
using namespace std::chrono_literals;
using namespace psen_scan_v2;
namespace standalone = psen_scan_v2_standalone;
namespace standalone_test = psen_scan_v2_standalone_test;

namespace psen_scan_v2_test
{
class IOSubscriberMock
{
public:
  IOSubscriberMock();
  MOCK_METHOD1(callback, void(const IOState&));

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};

inline IOSubscriberMock::IOSubscriberMock()
{
  subscriber_ = nh_.subscribe("/laser_1/io_state", 40, &IOSubscriberMock::callback, this);
}

class ActiveZoneSubscriberMock
{
public:
  ActiveZoneSubscriberMock();
  MOCK_METHOD1(callback, void(const std_msgs::UInt8&));

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};

inline ActiveZoneSubscriberMock::ActiveZoneSubscriberMock()
{
  subscriber_ = nh_.subscribe("/laser_1/active_zoneset", 1, &ActiveZoneSubscriberMock::callback, this);
}

class ZoneSwitchExecutor
{
public:
  ZoneSwitchExecutor();
  // Throw std::out_of_range if zone_id is not supported.
  void execute(unsigned int zone_id);

private:
  ros::NodeHandle nh_;
  ros::Publisher relay_cmd_publisher_;

private:
  static const std::map<unsigned int, uint8_t> ZONE_ID_TO_BYTE_CMD;
};

const std::map<unsigned int, uint8_t> ZoneSwitchExecutor::ZONE_ID_TO_BYTE_CMD{ { 1, 4 }, { 2, 12 } };

inline ZoneSwitchExecutor::ZoneSwitchExecutor()
{
  relay_cmd_publisher_ = nh_.advertise<std_msgs::Byte>("/relay_cmd", 1);
}

inline void ZoneSwitchExecutor::execute(unsigned int zone_id)
{
  std_msgs::Byte command;
  command.data = ZONE_ID_TO_BYTE_CMD.at(zone_id);
  relay_cmd_publisher_.publish(command);
}

MATCHER_P(UInt8MsgDataEq, data, "")
{
  return arg.data == data;
}

MATCHER_P(ZoneSwitchingInputIsTrue, zone_id, "")
{
  const std::string pin_name{ "Zone Set Switching Input " + std::to_string(zone_id) };
  bool pin_found{ false };
  for (const auto& pin : arg.physical_input)
  {
    if (pin.name == pin_name)
    {
      pin_found = true;
      if (!pin.state)
      {
        *result_listener << "Pin " << pin.name << " is not true";
        return false;
      }
    }
    else if (pin.state)
    {
      *result_listener << "Pin " << pin.name << " is true";
      return false;
    }
  }
  if (!pin_found)
  {
    *result_listener << "Pin " << pin_name << " not found";
  }
  return pin_found;
}

class IOStateTests : public Test
{
public:
  void SetUp() override
  {
    switchZone(1);
    ActiveZoneSubscriberMock active_zone_subscriber_mock;
    auto zone_one_barrier = EXPECT_ASYNC_CALL(active_zone_subscriber_mock, callback(UInt8MsgDataEq(1)));
    zone_one_barrier->waitTillRelease(5s);
  }

  void TearDown() override
  {
  }

  void switchZone(unsigned int zone_id)
  {
    zone_switch_executor_.execute(zone_id);
  }

protected:
  ZoneSwitchExecutor zone_switch_executor_;
};

TEST_F(IOStateTests, shouldPublishChangedZoneSwitchingIOIfZoneIsSwitched)
{
  standalone::util::Barrier zone_switching_1_barrier;
  standalone::util::Barrier zone_switching_2_barrier;

  IOSubscriberMock io_subscriber_mock;
  {
    InSequence s;
    EXPECT_CALL(io_subscriber_mock, callback(ZoneSwitchingInputIsTrue(2)))
        .Times(AnyNumber())
        .WillOnce(standalone_test::OpenBarrier(&zone_switching_2_barrier));
    EXPECT_CALL(io_subscriber_mock, callback(ZoneSwitchingInputIsTrue(1)))
        .Times(AnyNumber())
        .WillOnce(standalone_test::OpenBarrier(&zone_switching_1_barrier));
  }

  switchZone(2);
  zone_switching_2_barrier.waitTillRelease(5s);
  switchZone(1);
  zone_switching_1_barrier.waitTillRelease(5s);
}

}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "hwtest_io_state");
  ros::NodeHandle nh;

  // Needed since we use a subscriber
  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  return RUN_ALL_TESTS();
}
