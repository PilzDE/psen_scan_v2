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

#include <algorithm>
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

#include "psen_scan_v2/subscriber_mock.h"

namespace psen_scan_v2
{
// Avoids too much output with full io pin data
void PrintTo(const IOState& io_state, std::ostream* os)
{
  *os << "IOState(input:";
  for (const auto& input_pin : io_state.input)
  {
    *os << input_pin.name << ": " << (int)input_pin.state << ", ";
  }
  *os << "\noutput:";
  for (const auto& output_pin : io_state.output)
  {
    *os << output_pin.name << ": " << (int)output_pin.state << ", ";
  }
  *os << ")";
}
}  // namespace psen_scan_v2

using namespace ::testing;
using namespace std::chrono_literals;
using namespace psen_scan_v2;
namespace standalone = psen_scan_v2_standalone;
namespace standalone_test = psen_scan_v2_standalone_test;

namespace psen_scan_v2_test
{
class IOSubscriberMock : public SubscriberMock<IOState>
{
public:
  IOSubscriberMock(ros::NodeHandle& nh);
};

IOSubscriberMock::IOSubscriberMock(ros::NodeHandle& nh) : SubscriberMock<IOState>{ nh, "/laser_1/io_state", 6 }
{
}

class ZoneSwitchExecutor
{
public:
  ZoneSwitchExecutor();
  bool isConnected(const ros::Duration& timeout = ros::Duration(3.0)) const;
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

bool ZoneSwitchExecutor::isConnected(const ros::Duration& timeout) const
{
  const auto start_time = ros::Time::now();
  while (ros::ok())
  {
    if (relay_cmd_publisher_.getNumSubscribers() > 0)
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
  const auto it =
      std::find_if(arg.input.begin(), arg.input.end(), [pin_name](const auto& pin) { return pin.name == pin_name; });
  if (it == arg.input.end())
  {
    *result_listener << "Pin " << pin_name << " not found";
    return false;
  }
  return it->state == true;
}

MATCHER(OnlyOneZoneSwitchingInputIsTrue, "")
{
  return std::count_if(arg.input.begin(), arg.input.end(), [](const auto& pin) {
           return (pin.name.substr(0, 24) == "Zone Set Switching Input") && (pin.state == true);
         }) == 1;
}

MATCHER_P(Safety1IntrusionOutputIs, state, "")
{
  const auto it = std::find_if(
      arg.output.begin(), arg.output.end(), [&](const auto& pin) { return pin.name == "Safety 1 intrusion"; });
  if (it == arg.output.end())
  {
    *result_listener << "Pin Safety 1 intrusion not found";
    return false;
  }
  return it->state == state;
}

/**
 * For this test the scanner and it's configuration need to be set-up as follows:
 * - In zoneset 1 the safety1 zone is not violated
 * - In zoneset 2 the safety1 zone is violated
 */
class IOStateTests : public Test
{
public:
  void SetUp() override
  {
    IOSubscriberMock io_subscriber_mock(nh_);
    standalone::util::Barrier zone_one_barrier;
    ON_CALL(io_subscriber_mock, callback(AllOf(ZoneSwitchingInputIsTrue(1), Safety1IntrusionOutputIs(false))))
        .WillByDefault(standalone_test::OpenBarrier(&zone_one_barrier));

    ASSERT_TRUE(zone_switch_executor_.isConnected());
    switchZone(1);
    zone_one_barrier.waitTillRelease(5s);
  }

  void TearDown() override
  {
  }

  void switchZone(unsigned int zone_id)
  {
    zone_switch_executor_.execute(zone_id);
  }

protected:
  ros::NodeHandle nh_;
  ZoneSwitchExecutor zone_switch_executor_;
};

TEST_F(IOStateTests, shouldPublishChangedZoneSwitchingInputIfZoneIsSwitched)
{
  standalone::util::Barrier zone_switching_1_barrier;
  standalone::util::Barrier zone_switching_2_barrier;

  IOSubscriberMock io_subscriber_mock(nh_);
  {
    InSequence s;
    EXPECT_CALL(io_subscriber_mock,
                callback(AllOf(OnlyOneZoneSwitchingInputIsTrue(),
                               ZoneSwitchingInputIsTrue(1))))  // This is what we expect after SetUp()
        .Times(AtLeast(1))
        .WillOnce(standalone_test::OpenBarrier(&zone_switching_1_barrier));
    EXPECT_CALL(io_subscriber_mock, callback(AllOf(OnlyOneZoneSwitchingInputIsTrue(), ZoneSwitchingInputIsTrue(2))))
        .Times(AtLeast(1))
        .WillOnce(standalone_test::OpenBarrier(&zone_switching_2_barrier));
  }

  zone_switching_1_barrier.waitTillRelease(5s);
  switchZone(2);
  zone_switching_2_barrier.waitTillRelease(5s);
}

TEST_F(IOStateTests, shouldPublishChangedSafetyIntrusionOutputIfZoneIsSwitched)
{
  standalone::util::Barrier safety1_intrusion_false_barrier;
  standalone::util::Barrier safety1_intrusion_true_barrier;

  IOSubscriberMock io_subscriber_mock(nh_);
  {
    InSequence s;
    EXPECT_CALL(io_subscriber_mock, callback(Safety1IntrusionOutputIs(false)))  // This is what we expect after SetUp()
        .Times(AtLeast(1))
        .WillOnce(standalone_test::OpenBarrier(&safety1_intrusion_false_barrier));
    EXPECT_CALL(io_subscriber_mock, callback(Safety1IntrusionOutputIs(true)))
        .Times(AtLeast(1))
        .WillOnce(standalone_test::OpenBarrier(&safety1_intrusion_true_barrier));
  }

  safety1_intrusion_false_barrier.waitTillRelease(5s);
  switchZone(2);
  safety1_intrusion_true_barrier.waitTillRelease(5s);
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
