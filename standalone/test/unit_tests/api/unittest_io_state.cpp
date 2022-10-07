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

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_state_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"

#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data_helper.h"
#include "psen_scan_v2_standalone/util/assertions.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;
using data_conversion_layer::monitoring_frame::io::PinData;

TEST(IOStateTests, shouldReturnCorrectTimestamp)
{
  const IOState io_state{ PinData{}, 42 /*timestamp*/ };
  EXPECT_EQ(42, io_state.timestamp());
}

TEST(IOStateTests, shouldReturnInputsWhereAllAreUnsetWhenDefaultConstructed)
{
  const auto inputs{ IOState().input() };
  ASSERT_FALSE(inputs.empty());
  for (const auto& input : inputs)
  {
    EXPECT_FALSE(input.state());
  }
}

TEST(IOStateTests, shouldReturnOutputsWhereAllAreUnsetWhenDefaultConstructed)
{
  const auto outputs{ IOState().output() };
  ASSERT_FALSE(outputs.empty());
  for (const auto& output : outputs)
  {
    EXPECT_FALSE(output.state());
  }
}

TEST(IOStateTests, shouldReturnInputsEqualToConvertedInputPinData)
{
  PinData pin_data{};
  pin_data.input_state.at(5).set(1);
  ASSERT_TRUE(data_conversion_layer::isUsedInputBit(5, 1));

  const auto inputs{ IOState(pin_data, 0 /*timestamp*/).input() };
  EXPECT_EQ(inputs, data_conversion_layer::generateInputPinStates(pin_data));
}

TEST(IOStateTests, shouldReturnOutputsEqualToConvertedOutputPinData)
{
  PinData pin_data{};
  pin_data.output_state.at(0).set(2);
  ASSERT_TRUE(data_conversion_layer::isUsedOutputBit(0, 2));

  const auto outputs{ IOState(pin_data, 0 /*timestamp*/).output() };
  EXPECT_EQ(outputs, data_conversion_layer::generateOutputPinStates(pin_data));
}

TEST(IOStateTests, shouldNotBeEqualWithDifferentPinData)
{
  PinData pin_data{};
  pin_data.input_state.at(5).set(1);
  EXPECT_NE(IOState(pin_data, 0 /*timestamp*/), IOState(PinData{}, 0 /*timestamp*/));
}

TEST(IOStateTests, shouldBeEqualWhithEqualPinData)
{
  EXPECT_EQ(IOState(PinData{}, 0 /*timestamp*/), IOState(PinData{}, 0 /*timestamp*/));
}

TEST(IOStateTests, shouldBeEqualWithDifferentTimestamps)
{
  EXPECT_EQ(IOState(PinData{}, 0 /*timestamp*/), IOState(PinData{}, 42 /*timestamp*/));
}

TEST(IOStateTests, shouldBeEqualWhenDefaultConstructed)
{
  EXPECT_EQ(IOState{}, IOState{});
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
