// Copyright (c) 2022 Pilz GmbH & Co. KG
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

#include <gtest/gtest.h>

#include "psen_scan_v2/EncoderState.h"
#include "psen_scan_v2_standalone/encoder_state.h"
#include "psen_scan_v2_standalone/data_conversion_layer/encoder_data.h"

#include "psen_scan_v2/encoder_state_ros_conversion.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;
using data_conversion_layer::monitoring_frame::encoder::EncoderData;
namespace psen_scan_v2_test
{
TEST(EncoderStateROSConversionsTest, shouldConvertSuccessfully)
{
  EncoderData encoder_data{ 12.00 /*encoder 1*/, 25.876 /*encoder 2*/};
  psen_scan_v2_standalone::EncoderState encoder_state{ encoder_data, 56 /*timestamp*/ };

  EXPECT_NO_THROW(psen_scan_v2::EncoderState ros_message = toEncoderStateMsg(encoder_state, "some_frame"));
}

TEST(EncoderStateRosConversionsTest, shouldSetCorrectHeaderData)
{
  EncoderData encoder_data{ 16.40 /*encoder 1*/, 35.647 /*encoder 2*/};
  psen_scan_v2_standalone::EncoderState encoder_state(encoder_data , 10 /*timestamp*/);
  psen_scan_v2::EncoderState ros_message = toEncoderStateMsg(encoder_state, "some_frame");

  EXPECT_EQ(ros_message.header.stamp, ros::Time{}.fromNSec(10));
  EXPECT_EQ(ros_message.header.frame_id, "some_frame");
}

TEST(EncoderStateRosConversionsTest, shouldContainCorrectStates)
{
  EncoderData encoder_data{ 1.00 /*encoder 1*/, 45.876 /*encoder 2*/};
  psen_scan_v2_standalone::EncoderState encoder_state{encoder_data, 56 /*timestamp*/ };
  psen_scan_v2::EncoderState ros_message = toEncoderStateMsg(encoder_state, "some_frame");

  EXPECT_EQ(ros_message.encoder_1, encoder_state.getEncoder_1());
  EXPECT_EQ(ros_message.encoder_2, encoder_state.getEncoder_2());
}

TEST(EncoderStateRosConversionsTest, shouldThrowOnNegativeTime)
{
  EncoderData encoder_data{ 12.00 /*encoder 1*/, 25.876 /*encoder 2*/};
  psen_scan_v2_standalone::EncoderState encoder_state(encoder_data, -10 /*timestamp*/);
  EXPECT_THROW(toEncoderStateMsg(encoder_state, "some_frame"), std::invalid_argument);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
