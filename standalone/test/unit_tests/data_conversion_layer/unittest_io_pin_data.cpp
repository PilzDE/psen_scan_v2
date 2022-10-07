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

#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::io;

TEST(IOPinDataTest, shouldReturnCorrectInputType)
{
  for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
  {
    EXPECT_EQ(getInputType(0, bit_n), LogicalInputType::unused);
    EXPECT_EQ(getInputType(1, bit_n), LogicalInputType::unused);
    EXPECT_EQ(getInputType(2, bit_n), LogicalInputType::unused);
    EXPECT_EQ(getInputType(3, bit_n), LogicalInputType::unused);
  }

  EXPECT_EQ(getInputType(4, 0), LogicalInputType::muting_1_a);
  EXPECT_EQ(getInputType(4, 1), LogicalInputType::muting_2_a);
  EXPECT_EQ(getInputType(4, 2), LogicalInputType::unused);
  EXPECT_EQ(getInputType(4, 3), LogicalInputType::overr_1_a);
  EXPECT_EQ(getInputType(4, 4), LogicalInputType::overr_2_a);
  EXPECT_EQ(getInputType(4, 5), LogicalInputType::unused);
  EXPECT_EQ(getInputType(4, 6), LogicalInputType::zone_sw_1);
  EXPECT_EQ(getInputType(4, 7), LogicalInputType::zone_sw_2);

  EXPECT_EQ(getInputType(5, 0), LogicalInputType::zone_sw_3);
  EXPECT_EQ(getInputType(5, 1), LogicalInputType::zone_sw_4);
  EXPECT_EQ(getInputType(5, 2), LogicalInputType::zone_sw_5);
  EXPECT_EQ(getInputType(5, 3), LogicalInputType::zone_sw_6);
  EXPECT_EQ(getInputType(5, 4), LogicalInputType::zone_sw_7);
  EXPECT_EQ(getInputType(5, 5), LogicalInputType::zone_sw_8);
  EXPECT_EQ(getInputType(5, 6), LogicalInputType::reset_a);
  EXPECT_EQ(getInputType(5, 7), LogicalInputType::unused);

  EXPECT_EQ(getInputType(6, 0), LogicalInputType::restart_1_a);
  EXPECT_EQ(getInputType(6, 1), LogicalInputType::mut_en_1_a);
  EXPECT_EQ(getInputType(6, 2), LogicalInputType::cor_seq_mut_1);
  EXPECT_EQ(getInputType(6, 3), LogicalInputType::cor_seq_or_1);
  EXPECT_EQ(getInputType(6, 4), LogicalInputType::unused);
  EXPECT_EQ(getInputType(6, 5), LogicalInputType::restart_2_a);
  EXPECT_EQ(getInputType(6, 6), LogicalInputType::mut_en_2_a);
  EXPECT_EQ(getInputType(6, 7), LogicalInputType::cor_seq_mut_2);

  EXPECT_EQ(getInputType(7, 0), LogicalInputType::cor_seq_or_2);
  for (std::size_t bit_n = 1; bit_n < 8; ++bit_n)
  {
    EXPECT_EQ(getInputType(7, bit_n), LogicalInputType::unused);
  }
}

TEST(IOPinDataTest, shouldReturnCorrectInputName)
{
  for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
  {
    EXPECT_EQ(getInputName(0, bit_n), "unused");
    EXPECT_EQ(getInputName(1, bit_n), "unused");
    EXPECT_EQ(getInputName(2, bit_n), "unused");
    EXPECT_EQ(getInputName(3, bit_n), "unused");
  }

  EXPECT_EQ(getInputName(4, 0), "Muting 1 Activated");
  EXPECT_EQ(getInputName(4, 1), "Muting 2 Activated");
  EXPECT_EQ(getInputName(4, 2), "unused");
  EXPECT_EQ(getInputName(4, 3), "Override 1 Activated");
  EXPECT_EQ(getInputName(4, 4), "Override 2 Activated");
  EXPECT_EQ(getInputName(4, 5), "unused");
  EXPECT_EQ(getInputName(4, 6), "Zone Set Switching Input 1");
  EXPECT_EQ(getInputName(4, 7), "Zone Set Switching Input 2");

  EXPECT_EQ(getInputName(5, 0), "Zone Set Switching Input 3");
  EXPECT_EQ(getInputName(5, 1), "Zone Set Switching Input 4");
  EXPECT_EQ(getInputName(5, 2), "Zone Set Switching Input 5");
  EXPECT_EQ(getInputName(5, 3), "Zone Set Switching Input 6");
  EXPECT_EQ(getInputName(5, 4), "Zone Set Switching Input 7");
  EXPECT_EQ(getInputName(5, 5), "Zone Set Switching Input 8");
  EXPECT_EQ(getInputName(5, 6), "Reset Activated");
  EXPECT_EQ(getInputName(5, 7), "unused");

  EXPECT_EQ(getInputName(6, 0), "Restart 1 Activated");
  EXPECT_EQ(getInputName(6, 1), "Muting Enable 1 Activated");
  EXPECT_EQ(getInputName(6, 2), "Correct activation sequence of Muting 1 Pins");
  EXPECT_EQ(getInputName(6, 3), "Correct activation sequence of Override 1 Pins");
  EXPECT_EQ(getInputName(6, 4), "unused");
  EXPECT_EQ(getInputName(6, 5), "Restart 2 Activated");
  EXPECT_EQ(getInputName(6, 6), "Muting Enable 2 Activated");
  EXPECT_EQ(getInputName(6, 7), "Correct activation sequence of Muting 2 Pins");

  EXPECT_EQ(getInputName(7, 0), "Correct activation sequence of Override 2 Pins");
  for (std::size_t bit_n = 1; bit_n < 8; ++bit_n)
  {
    EXPECT_EQ(getInputName(7, bit_n), "unused");
  }
}

TEST(IOPinDataTest, shouldReturnCorrectOutputType)
{
  EXPECT_EQ(getOutputType(0, 0), OutputType::safe_1_int);
  EXPECT_EQ(getOutputType(0, 1), OutputType::int_lock_1);
  EXPECT_EQ(getOutputType(0, 2), OutputType::safe_2_int);
  EXPECT_EQ(getOutputType(0, 3), OutputType::int_lock_2);
  EXPECT_EQ(getOutputType(0, 4), OutputType::safe_3_int);
  EXPECT_EQ(getOutputType(0, 5), OutputType::unused);
  EXPECT_EQ(getOutputType(0, 6), OutputType::warn_1_int);
  EXPECT_EQ(getOutputType(0, 7), OutputType::warn_2_int);

  for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
  {
    EXPECT_EQ(getOutputType(1, bit_n), OutputType::unused);
    EXPECT_EQ(getOutputType(2, bit_n), OutputType::unused);
  }

  EXPECT_EQ(getOutputType(3, 0), OutputType::unused);
  EXPECT_EQ(getOutputType(3, 1), OutputType::unused);
  EXPECT_EQ(getOutputType(3, 2), OutputType::unused);
  EXPECT_EQ(getOutputType(3, 3), OutputType::unused);
  EXPECT_EQ(getOutputType(3, 4), OutputType::ossd1_refpts);
  EXPECT_EQ(getOutputType(3, 5), OutputType::unused);
  EXPECT_EQ(getOutputType(3, 6), OutputType::unused);
  EXPECT_EQ(getOutputType(3, 7), OutputType::unused);
}

TEST(IOPinDataTest, shouldReturnCorrectOutputName)
{
  EXPECT_EQ(getOutputName(0, 0), "Safety 1 intrusion");
  EXPECT_EQ(getOutputName(0, 1), "INTERLOCK 1");
  EXPECT_EQ(getOutputName(0, 2), "Safety 2 intrusion");
  EXPECT_EQ(getOutputName(0, 3), "INTERLOCK 2");
  EXPECT_EQ(getOutputName(0, 4), "Safety 3 intrusion");
  EXPECT_EQ(getOutputName(0, 5), "unused");
  EXPECT_EQ(getOutputName(0, 6), "Warning 1 intrusion");
  EXPECT_EQ(getOutputName(0, 7), "Warning 2 intrusion");

  for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
  {
    EXPECT_EQ(getOutputName(1, bit_n), "unused");
    EXPECT_EQ(getOutputName(2, bit_n), "unused");
  }

  EXPECT_EQ(getOutputName(3, 0), "unused");
  EXPECT_EQ(getOutputName(3, 1), "unused");
  EXPECT_EQ(getOutputName(3, 2), "unused");
  EXPECT_EQ(getOutputName(3, 3), "unused");
  EXPECT_EQ(getOutputName(3, 4), "REFERENCE POINTS VIOLATION");
  EXPECT_EQ(getOutputName(3, 5), "unused");
  EXPECT_EQ(getOutputName(3, 6), "unused");
  EXPECT_EQ(getOutputName(3, 7), "unused");
}

TEST(IOPinDataTest, shouldReturnFalseInputPinStatesAfterConstruction)
{
  PinData pin_data{};
  for (std::size_t byte_n = 0; byte_n < NUMBER_OF_INPUT_BYTES; ++byte_n)
  {
    EXPECT_BITSETS_EQ(pin_data.input_state.at(byte_n), std::bitset<8>(0x00));
  }
}

TEST(IOPinDataTest, shouldSetOnlySpecifiedInputPinState)
{
  PinData pin_data{};
  pin_data.input_state.at(4).set(3);
  for (std::size_t byte_n = 0; byte_n < NUMBER_OF_INPUT_BYTES; ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      EXPECT_EQ(pin_data.input_state.at(byte_n).test(bit_n), byte_n == 4 && bit_n == 3);
    }
  }
}

TEST(IOPinDataTest, shouldReturnFalseOutputPinStatesAfterConstruction)
{
  PinData pin_data{};
  for (std::size_t byte_n = 0; byte_n < NUMBER_OF_OUTPUT_BYTES; ++byte_n)
  {
    EXPECT_BITSETS_EQ(pin_data.output_state.at(byte_n), std::bitset<8>(0x00));
  }
}

TEST(IOPinDataTest, shouldSetOnlySpecifiedOutputPinState)
{
  PinData pin_data{};
  pin_data.output_state.at(2).set(3);
  for (std::size_t byte_n = 0; byte_n < NUMBER_OF_OUTPUT_BYTES; ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      EXPECT_EQ(pin_data.output_state.at(byte_n).test(bit_n), byte_n == 2 && bit_n == 3);
    }
  }
}

TEST(IOPinDataTest, shouldCorrectlyWriteToInputStateArray)
{
  PinData pin_data{};
  pin_data.input_state.at(4).set(3);
  for (std::size_t byte_n = 0; byte_n < NUMBER_OF_INPUT_BYTES; ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      EXPECT_EQ(pin_data.input_state.at(byte_n).test(bit_n), byte_n == 4 && bit_n == 3);
    }
  }
}

TEST(IOPinDataTest, shouldCorrectlyWriteToOutputStateArray)
{
  PinData pin_data{};
  pin_data.output_state.at(2).set(3);
  for (std::size_t byte_n = 0; byte_n < NUMBER_OF_OUTPUT_BYTES; ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      EXPECT_EQ(pin_data.output_state.at(byte_n).test(bit_n), byte_n == 2 && bit_n == 3);
    }
  }
}

TEST(IOPinDataTest, shouldNotBeEqualWithDifferentInputState)
{
  PinData pin_data{};
  pin_data.input_state.at(4).set(3);
  EXPECT_FALSE(pin_data == PinData{});
}

TEST(IOPinDataTest, shouldNotBeEqualWithDifferentOutputState)
{
  PinData pin_data{};
  pin_data.output_state.at(2).set(3);
  EXPECT_FALSE(pin_data == PinData{});
}

TEST(IOPinDataTest, shouldBeEqual)
{
  EXPECT_TRUE(PinData{} == PinData{});
}
}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
