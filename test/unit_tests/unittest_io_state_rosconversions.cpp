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

#include <gtest/gtest.h>
#include "psen_scan_v2/IOState.h"

#include "psen_scan_v2/io_state_ros_conversion.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
TEST(IOStateROSConversionsTest, shouldConvertSuccessfully)
{
  psen_scan_v2_standalone::IOState iostate({ psen_scan_v2_standalone::PinState(1 /*id*/, "logical_input1", true),
                                             psen_scan_v2_standalone::PinState(2 /*id*/, "logical_input2", false) },
                                           { psen_scan_v2_standalone::PinState(3 /*id*/, "output1", true),
                                             psen_scan_v2_standalone::PinState(4 /*id*/, "output2", false) });
  psen_scan_v2::IOState ros_message = toIOStateMsg(iostate, "some_frame", 10 /* stamp */);

  EXPECT_EQ(ros_message.header.stamp, ros::Time{}.fromNSec(10));
  EXPECT_EQ(ros_message.header.frame_id, "some_frame");

  ASSERT_EQ(ros_message.logical_input.size(), 2u);

  EXPECT_EQ(ros_message.logical_input[0].pin_id, 1);
  EXPECT_EQ(ros_message.logical_input[0].name, "logical_input1");
  EXPECT_EQ(ros_message.logical_input[0].state, true);

  EXPECT_EQ(ros_message.logical_input[1].pin_id, 2);
  EXPECT_EQ(ros_message.logical_input[1].name, "logical_input2");
  EXPECT_EQ(ros_message.logical_input[1].state, false);

  ASSERT_EQ(ros_message.output.size(), 2u);

  EXPECT_EQ(ros_message.output[0].pin_id, 3);
  EXPECT_EQ(ros_message.output[0].name, "output1");
  EXPECT_EQ(ros_message.output[0].state, true);

  EXPECT_EQ(ros_message.output[1].pin_id, 4);
  EXPECT_EQ(ros_message.output[1].name, "output2");
  EXPECT_EQ(ros_message.output[1].state, false);
}

TEST(IOStateROSConversionsTest, shouldThrowOnNegativeTime)
{
  psen_scan_v2_standalone::IOState iostate({}, {});
  EXPECT_THROW(toIOStateMsg(iostate, "some_frame", -10), std::invalid_argument);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
