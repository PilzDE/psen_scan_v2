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

namespace psen_scan_v2_standalone_test
{
using psen_scan_v2_standalone::IOState;
using psen_scan_v2_standalone::PinState;

static const PinState PIN_STATE(0, "", false);

TEST(PinStateTests, shouldReturnInput0WithPinState)
{
  IOState io_state({ PIN_STATE }, {}, {}, {}, {});
  EXPECT_EQ(io_state.physicalInput0().at(0), PIN_STATE);
  EXPECT_TRUE(io_state.physicalInput1().empty());
  EXPECT_TRUE(io_state.physicalInput2().empty());
  EXPECT_TRUE(io_state.logicalInput().empty());
  EXPECT_TRUE(io_state.output().empty());
}

TEST(PinStateTests, shouldReturnInput1WithPinState)
{
  IOState io_state({}, { PIN_STATE }, {}, {}, {});
  EXPECT_TRUE(io_state.physicalInput0().empty());
  EXPECT_EQ(io_state.physicalInput1().at(0), PIN_STATE);
  EXPECT_TRUE(io_state.physicalInput2().empty());
  EXPECT_TRUE(io_state.logicalInput().empty());
  EXPECT_TRUE(io_state.output().empty());
}

TEST(PinStateTests, shouldReturnInput2WithPinState)
{
  IOState io_state({}, {}, { PIN_STATE }, {}, {});
  EXPECT_TRUE(io_state.physicalInput0().empty());
  EXPECT_TRUE(io_state.physicalInput1().empty());
  EXPECT_EQ(io_state.physicalInput2().at(0), PIN_STATE);
  EXPECT_TRUE(io_state.logicalInput().empty());
  EXPECT_TRUE(io_state.output().empty());
}

TEST(PinStateTests, shouldReturnLogicalInputWithPinState)
{
  IOState io_state({}, {}, {}, { PIN_STATE }, {});
  EXPECT_TRUE(io_state.physicalInput0().empty());
  EXPECT_TRUE(io_state.physicalInput1().empty());
  EXPECT_TRUE(io_state.physicalInput2().empty());
  EXPECT_EQ(io_state.logicalInput().at(0), PIN_STATE);
  EXPECT_TRUE(io_state.output().empty());
}

TEST(PinStateTests, shouldReturnOutputWithPinState)
{
  IOState io_state({}, {}, {}, {}, { PIN_STATE });
  EXPECT_TRUE(io_state.physicalInput0().empty());
  EXPECT_TRUE(io_state.physicalInput1().empty());
  EXPECT_TRUE(io_state.physicalInput2().empty());
  EXPECT_TRUE(io_state.logicalInput().empty());
  EXPECT_EQ(io_state.output().at(0), PIN_STATE);
}
}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}