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

#include <string>
#include <algorithm>
#include <gtest/gtest.h>
#include "psen_scan_v2/IOState.h"
#include "psen_scan_v2/InputPinID.h"
#include "psen_scan_v2/OutputPinID.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data_helper.h"

#include "psen_scan_v2/io_state_ros_conversion.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone_test;
using psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::io::PinData;
namespace psen_scan_v2_test
{
TEST(IOStateROSConversionsTest, shouldConvertSuccessfully)
{
  PinData pin_data{};
  setInputBit(pin_data, InputPinID::MUTING_1_ACTIVE);
  setOutputBit(pin_data, OutputPinID::SAFETY_1_INTRUSION);
  psen_scan_v2_standalone::IOState iostate(pin_data);

  EXPECT_NO_THROW(psen_scan_v2::IOState ros_message = toIOStateMsg(iostate, "some_frame", 10 /* stamp */));
}

TEST(IOStateRosConversionsTest, shouldSetCorrectHeaderData)
{
  psen_scan_v2_standalone::IOState io_state(PinData{});
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame", 10 /* stamp */);

  EXPECT_EQ(ros_message.header.stamp, ros::Time{}.fromNSec(10));
  EXPECT_EQ(ros_message.header.frame_id, "some_frame");
}

TEST(IOStateRosConversionsTest, shouldAddTheCorrectNumberOfInputStates)
{
  psen_scan_v2_standalone::IOState io_state(PinData{});
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame", 10 /* stamp */);

  ASSERT_EQ(ros_message.input.size(), 29u);
}

TEST(IOStateRosConversionsTest, shouldAddTheCorrectNumberOfOutputStates)
{
  psen_scan_v2_standalone::IOState io_state(PinData{});
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame", 10 /* stamp */);
  ASSERT_EQ(ros_message.output.size(), 8u);
}

TEST(IOStateRosConversionsTest, shouldAddAllInputStatesOnce)
{
  psen_scan_v2_standalone::IOState io_state(PinData{});
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame", 10 /* stamp */);

  for (const auto& pin : io_state.input())
  {
    EXPECT_EQ(std::count_if(ros_message.input.begin(),
                            ros_message.input.end(),
                            [&pin](const auto& i) { return static_cast<uint32_t>(i.pin_id.id) == pin.id(); }),
              1)
        << "Wrong number of inputs with id " << pin.id() << " in the resulting Message";
  }
}

TEST(IOStateRosConversionsTest, shouldAddAllOutputStatesOnce)
{
  psen_scan_v2_standalone::IOState io_state(PinData{});
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame", 10 /* stamp */);

  for (const auto& pin : io_state.output())
  {
    EXPECT_EQ(std::count_if(ros_message.output.begin(),
                            ros_message.output.end(),
                            [&pin](const auto& i) { return static_cast<uint32_t>(i.pin_id.id) == pin.id(); }),
              1)
        << "Wrong number of outputs with id " << pin.id() << " in the resulting Message";
  }
}

TEST(IOStateRosConversionsTest, shouldContainCorrectInputStates)
{
  PinData pin_data{};
  setInputBit(pin_data, InputPinID::MUTING_1_ACTIVE);
  psen_scan_v2_standalone::IOState io_state(pin_data);
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame", 10 /* stamp */);

  for (const auto& pin : io_state.input())
  {
    auto it = std::find_if(ros_message.input.begin(), ros_message.input.end(), [&pin](const auto& i) {
      return static_cast<uint32_t>(i.pin_id.id) == pin.id();
    });
    ASSERT_NE(it, ros_message.input.end());
    EXPECT_EQ(it->name, pin.name());
    EXPECT_EQ(it->state, pin.state());
  }
}

TEST(IOStateRosConversionsTest, shouldContainCorrectOutputStates)
{
  PinData pin_data{};
  setInputBit(pin_data, OutputPinID::SAFETY_1_INTRUSION);
  psen_scan_v2_standalone::IOState io_state(pin_data);
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame", 10 /* stamp */);

  for (const auto& pin : io_state.output())
  {
    auto it = std::find_if(ros_message.output.begin(), ros_message.output.end(), [&pin](const auto& i) {
      return static_cast<uint32_t>(i.pin_id.id) == pin.id();
    });
    ASSERT_NE(it, ros_message.output.end());
    EXPECT_EQ(it->name, pin.name());
    EXPECT_EQ(it->state, pin.state());
  }
}

TEST(IOStateROSConversionsTest, shouldThrowOnNegativeTime)
{
  psen_scan_v2_standalone::IOState io_state(PinData{});
  EXPECT_THROW(toIOStateMsg(io_state, "some_frame", -10), std::invalid_argument);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
