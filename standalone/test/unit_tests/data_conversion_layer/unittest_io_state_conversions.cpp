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
#include <vector>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_state_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"

#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data_helper.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;
using data_conversion_layer::monitoring_frame::io::PinData;

TEST(IOStateConversionsTest, shouldReturnInputPinStateWithCorrectId)
{
  const auto pin_state{ data_conversion_layer::generateInputPinState(PinData(), 4, 3) };
  EXPECT_EQ(pin_state.id(), 35U);
}

TEST(IOStateConversionsTest, shouldReturnInputPinStateWithCorrectName)
{
  const auto pin_state{ data_conversion_layer::generateInputPinState(PinData(), 4, 3) };
  EXPECT_EQ(pin_state.name(), data_conversion_layer::monitoring_frame::io::getInputName(4, 3));
}

TEST(IOStateConversionsTest, shouldReturnInputPinStateWithCorrectState)
{
  const auto pin_state1{ data_conversion_layer::generateInputPinState(PinData(), 4, 3) };
  EXPECT_FALSE(pin_state1.state());

  PinData pin_data;
  pin_data.inputPinState(4, 3, true);
  const auto pin_state2{ data_conversion_layer::generateInputPinState(pin_data, 4, 3) };
  EXPECT_TRUE(pin_state2.state());
}

TEST(IOStateConversionsTest, shouldReturnOutputPinStateWithCorrectId)
{
  const auto pin_state{ data_conversion_layer::generateOutputPinState(PinData(), 1, 3) };
  EXPECT_EQ(pin_state.id(), 11U);
}

TEST(IOStateConversionsTest, shouldReturnOutputPinStateWithCorrectName)
{
  const auto pin_state{ data_conversion_layer::generateOutputPinState(PinData(), 0, 3) };
  EXPECT_EQ(pin_state.name(), data_conversion_layer::monitoring_frame::io::getOutputName(0, 3));
}

TEST(IOStateConversionsTest, shouldReturnOutputPinStateWithCorrectState)
{
  const auto pin_state1{ data_conversion_layer::generateOutputPinState(PinData(), 0, 3) };
  EXPECT_FALSE(pin_state1.state());

  PinData pin_data;
  pin_data.outputPinState(0, 3, true);
  const auto pin_state2{ data_conversion_layer::generateOutputPinState(pin_data, 0, 3) };
  EXPECT_TRUE(pin_state2.state());
}

TEST(IOStateConversionsTest, shouldReturnInputPinStatesEqualToIndividuallyGeneratedPinStates)
{
  const auto pin_data{ createPinData() };
  const auto pin_states{ data_conversion_layer::generateInputPinStates(pin_data) };

  ASSERT_FALSE(pin_states.empty());
  for (const auto& pin_state : pin_states)
  {
    EXPECT_EQ(
        pin_state,
        data_conversion_layer::generateInputPinState(pin_data, idToByte(pin_state.id()), idToBit(pin_state.id())));
  }
}

uint32_t createId(std::size_t byte, std::size_t bit)
{
  return static_cast<uint32_t>(byte * 8 + bit);
}

TEST(IOStateConversionsTest, shouldReturnInputPinStatesForAllUsedInputsWithoutRepetition)
{
  const auto pin_states{ data_conversion_layer::generateInputPinStates(PinData()) };
  for (std::size_t byte_n = 0; byte_n < data_conversion_layer::monitoring_frame::io::NUMBER_OF_INPUT_BYTES; ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      if (data_conversion_layer::monitoring_frame::io::getInputType(byte_n, bit_n) !=
          data_conversion_layer::monitoring_frame::io::LogicalInputType::unused)
      {
        EXPECT_EQ(std::count_if(pin_states.begin(),
                                pin_states.end(),
                                [&](const auto& pin_state) { return pin_state.id() == createId(byte_n, bit_n); }),
                  1)
            << "Wrong number of elements with id " << createId(byte_n, bit_n);
      }
    }
  }
}

TEST(IOStateConversionsTest, shouldReturnOutputPinStatesEqualToIndividuallyGeneratedPinStates)
{
  const auto pin_data{ createPinData() };
  const auto pin_states{ data_conversion_layer::generateOutputPinStates(pin_data) };

  ASSERT_FALSE(pin_states.empty());
  for (const auto& pin_state : pin_states)
  {
    EXPECT_EQ(
        pin_state,
        data_conversion_layer::generateOutputPinState(pin_data, idToByte(pin_state.id()), idToBit(pin_state.id())));
  }
}

TEST(IOStateConversionsTest, shouldReturnOutputPinStatesForAllUsedOutputsWithoutRepetition)
{
  const auto pin_states{ data_conversion_layer::generateOutputPinStates(PinData()) };
  for (std::size_t byte_n = 0; byte_n < data_conversion_layer::monitoring_frame::io::NUMBER_OF_OUTPUT_BYTES; ++byte_n)
  {
    for (std::size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      if (data_conversion_layer::monitoring_frame::io::getOutputType(byte_n, bit_n) !=
          data_conversion_layer::monitoring_frame::io::OutputType::unused)
      {
        EXPECT_EQ(std::count_if(pin_states.begin(),
                                pin_states.end(),
                                [&](const auto& pin_state) { return pin_state.id() == createId(byte_n, bit_n); }),
                  1)
            << "Wrong number of elements with id " << createId(byte_n, bit_n);
      }
    }
  }
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
