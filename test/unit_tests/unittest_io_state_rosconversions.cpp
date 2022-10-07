// Copyright (c) 2021-2022 Pilz GmbH & Co. KG
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
  psen_scan_v2_standalone::IOState iostate(pin_data, 9 /*timestamp*/);

  EXPECT_NO_THROW(psen_scan_v2::IOState ros_message = toIOStateMsg(iostate, "some_frame"));
}

TEST(IOStateRosConversionsTest, shouldSetCorrectHeaderData)
{
  psen_scan_v2_standalone::IOState io_state(PinData{}, 10 /*timestamp*/);
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame");

  EXPECT_EQ(ros_message.header.stamp, ros::Time{}.fromNSec(10));
  EXPECT_EQ(ros_message.header.frame_id, "some_frame");
}

TEST(IOStateRosConversionsTest, shouldAddTheCorrectNumberOfInputStates)
{
  psen_scan_v2_standalone::IOState io_state{};
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame");

  ASSERT_EQ(ros_message.input.size(), 21u);
}

TEST(IOStateRosConversionsTest, shouldAddTheCorrectNumberOfOutputStates)
{
  psen_scan_v2_standalone::IOState io_state{};
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame");
  ASSERT_EQ(ros_message.output.size(), 8u);
}

template <typename PinState>
std::size_t countROSPinStatesWithId(const std::vector<PinState>& pin_states, uint32_t id)
{
  return std::count_if(pin_states.begin(), pin_states.end(), [id](const auto& pin_state) {
    return static_cast<uint32_t>(pin_state.pin_id.id) == id;
  });
}

template <typename PinState>
typename std::vector<PinState>::const_iterator findROSPinStateWithId(const std::vector<PinState>& pin_states,
                                                                     uint32_t id)
{
  return std::find_if(pin_states.begin(), pin_states.end(), [id](const auto& pin_state) {
    return static_cast<uint32_t>(pin_state.pin_id.id) == id;
  });
}

TEST(IOStateRosConversionsTest, shouldAddAllInputStatesOnce)
{
  psen_scan_v2_standalone::IOState io_state{};
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame");

  for (const auto& pin : io_state.input())
  {
    const auto pin_state_count{ countROSPinStatesWithId(ros_message.input, pin.id()) };
    EXPECT_EQ(pin_state_count, 1u) << "Wrong number of inputs with id " << pin.id() << " in the resulting Message";
  }
}

TEST(IOStateRosConversionsTest, shouldAddAllOutputStatesOnce)
{
  psen_scan_v2_standalone::IOState io_state{};
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame");

  for (const auto& pin : io_state.output())
  {
    const auto pin_state_count{ countROSPinStatesWithId(ros_message.output, pin.id()) };
    EXPECT_EQ(pin_state_count, 1u) << "Wrong number of outputs with id " << pin.id() << " in the resulting Message";
  }
}

TEST(IOStateRosConversionsTest, shouldContainCorrectInputStates)
{
  PinData pin_data{};
  setInputBit(pin_data, InputPinID::MUTING_1_ACTIVE);
  psen_scan_v2_standalone::IOState io_state(pin_data, 42 /*timestamp*/);
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame");

  for (const auto& pin : io_state.input())
  {
    const auto it{ findROSPinStateWithId(ros_message.input, pin.id()) };
    ASSERT_NE(it, ros_message.input.end());
    EXPECT_EQ(it->name, pin.name());
    EXPECT_EQ(it->state, pin.state());
  }
}

TEST(IOStateRosConversionsTest, shouldContainCorrectOutputStates)
{
  PinData pin_data{};
  setInputBit(pin_data, OutputPinID::SAFETY_1_INTRUSION);
  psen_scan_v2_standalone::IOState io_state(pin_data, 42 /*timestamp*/);
  psen_scan_v2::IOState ros_message = toIOStateMsg(io_state, "some_frame");

  for (const auto& pin : io_state.output())
  {
    const auto it{ findROSPinStateWithId(ros_message.output, pin.id()) };
    ASSERT_NE(it, ros_message.output.end());
    EXPECT_EQ(it->name, pin.name());
    EXPECT_EQ(it->state, pin.state());
  }
}

TEST(IOStateROSConversionsTest, shouldThrowOnNegativeTime)
{
  psen_scan_v2_standalone::IOState io_state(PinData{}, -10 /*timestamp*/);
  EXPECT_THROW(toIOStateMsg(io_state, "some_frame"), std::invalid_argument);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
