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

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/encoder_state.h"
#include "psen_scan_v2_standalone/data_conversion_layer/encoder_data.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;
using data_conversion_layer::monitoring_frame::encoder::EncoderData;

TEST(EncoderStateTests, shouldReturnCorrectTimestamp)
{
  const EncoderState encoder_state{ EncoderData{}, 42 /*timestamp*/ };
  EXPECT_EQ(42, encoder_state.timestamp());
}

TEST(EncoderStateTests, shouldReturnCorrectValueForEncoder1)
{
  const EncoderState encoder_state{ EncoderData{ 12.0005 /*encoder_1*/, 25.786 /*encoder_2*/ }, 42 /*timestamp*/ };
  const auto encoder_1 = encoder_state.getEncoder1();
  EXPECT_DOUBLE_EQ(encoder_1, 12.0005);
}

TEST(EncoderStateTests, shouldReturnCorrectValueForEncoder2)
{
  const EncoderState encoder_state{ EncoderData{ 12.0005 /*encoder_1*/, 25.786 /*encoder_2*/ }, 42 /*timestamp*/ };
  const auto encoder_2 = encoder_state.getEncoder2();
  EXPECT_DOUBLE_EQ(encoder_2, 25.786);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
