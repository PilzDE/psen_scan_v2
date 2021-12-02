// Copyright (c) 2019-2021 Pilz GmbH & Co. KG
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
#include "psen_scan_v2_standalone/data_conversion_layer/io_pin_data.h"

namespace psen_scan_v2_standalone_test
{
using psen_scan_v2_standalone::PinState;
using namespace psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::io;

TEST(PinStateTests, shouldReturnCorrectInputPinState)
{
  for (size_t byte_n = 0; byte_n < RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES; byte_n++)
  {
    for (size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      const auto id = byte_n * 8 + bit_n;
      const auto& name = PHYSICAL_INPUT_BIT_TO_NAME.at(PHYSICAL_INPUT_BITS.at(byte_n).at(bit_n));
      EXPECT_EQ(createInputPinState(byte_n, bit_n, true), PinState(id, name, true));
    }
  }
}

TEST(PinStateTests, shouldReturnCorrectOutputPinState)
{
  for (size_t byte_n = 0; byte_n < RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES; byte_n++)
  {
    for (size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      const auto id = byte_n * 8 + bit_n;
      const auto& name = OUTPUT_BIT_TO_NAME.at(OUTPUT_BITS.at(byte_n).at(bit_n));
      EXPECT_EQ(createOutputPinState(byte_n, bit_n, true), PinState(id, name, true));
    }
  }
}

TEST(PinStateTests, shouldReturnCorrectLogicalPinState)
{
  for (size_t byte_n = 0; byte_n < RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES; byte_n++)
  {
    for (size_t bit_n = 0; bit_n < 8; ++bit_n)
    {
      const auto id = byte_n * 8 + bit_n;
      const auto& name = LOGICAL_INPUT_BIT_TO_NAME.at(LOGICAL_INPUT_BITS.at(byte_n).at(bit_n));
      EXPECT_EQ(createLogicalPinState(byte_n, bit_n, true), PinState(id, name, true));
    }
  }
}

TEST(PinStateTests, shouldThrowOnInvalidInputPositions)
{
  EXPECT_THROW(createInputPinState(RAW_CHUNK_PHYSICAL_INPUT_SIGNALS_IN_BYTES, 0, false), std::out_of_range);
  EXPECT_THROW(createInputPinState(-1, 0, false), std::out_of_range);
  EXPECT_THROW(createInputPinState(0, 9, false), std::out_of_range);
  EXPECT_THROW(createInputPinState(0, -1, false), std::out_of_range);
}

TEST(PinStateTests, shouldThrowOnInvalidOutputPositions)
{
  EXPECT_THROW(createOutputPinState(RAW_CHUNK_OUTPUT_SIGNALS_IN_BYTES, 0, false), std::out_of_range);
  EXPECT_THROW(createOutputPinState(-1, 0, false), std::out_of_range);
  EXPECT_THROW(createOutputPinState(0, 9, false), std::out_of_range);
  EXPECT_THROW(createOutputPinState(0, -1, false), std::out_of_range);
}

TEST(PinStateTests, shouldThrowOnInvalidLogicalInputPositions)
{
  EXPECT_THROW(createLogicalPinState(RAW_CHUNK_LOGICAL_INPUT_SIGNALS_IN_BYTES, 0, false), std::out_of_range);
  EXPECT_THROW(createLogicalPinState(-1, 0, false), std::out_of_range);
  EXPECT_THROW(createLogicalPinState(0, 9, false), std::out_of_range);
  EXPECT_THROW(createLogicalPinState(0, -1, false), std::out_of_range);
}
}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}