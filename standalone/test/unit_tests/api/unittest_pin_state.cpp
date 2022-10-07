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

#include <string>

#include <gtest/gtest.h>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/io_state.h"

namespace psen_scan_v2_standalone_test
{
using psen_scan_v2_standalone::PinState;

static const uint32_t EXPECTED_ID_1 = 5;
static const uint32_t EXPECTED_ID_2 = 11;
static const std::string EXPECTED_NAME_1 = "unused";
static const std::string EXPECTED_NAME_2 = "M_OSSD";
static const bool EXPECTED_STATE_1 = false;
static const bool EXPECTED_STATE_2 = true;

TEST(PinStateTests, shouldReturnSetId)
{
  PinState pin1(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_1);
  PinState pin2(EXPECTED_ID_2, EXPECTED_NAME_1, EXPECTED_STATE_1);
  EXPECT_EQ(pin1.id(), EXPECTED_ID_1);
  EXPECT_EQ(pin2.id(), EXPECTED_ID_2);
}

TEST(PinStateTests, shouldReturnSetName)
{
  PinState pin1(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_1);
  PinState pin2(EXPECTED_ID_1, EXPECTED_NAME_2, EXPECTED_STATE_1);
  EXPECT_EQ(pin1.name(), EXPECTED_NAME_1);
  EXPECT_EQ(pin2.name(), EXPECTED_NAME_2);
}

TEST(PinStateTests, shouldReturnSetState)
{
  PinState pin1(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_1);
  PinState pin2(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_2);
  EXPECT_EQ(pin1.state(), EXPECTED_STATE_1);
  EXPECT_EQ(pin2.state(), EXPECTED_STATE_2);
}

TEST(PinStateTests, shouldNotBeEqualWithDifferentId)
{
  PinState pin1(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_1);
  PinState pin2(EXPECTED_ID_2, EXPECTED_NAME_1, EXPECTED_STATE_1);
  EXPECT_NE(pin1, pin2);
}

TEST(PinStateTests, shouldNotBeEqualWithDifferentName)
{
  PinState pin1(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_1);
  PinState pin2(EXPECTED_ID_1, EXPECTED_NAME_2, EXPECTED_STATE_1);
  EXPECT_NE(pin1, pin2);
}

TEST(PinStateTests, shouldNotBeEqualWithDifferentState)
{
  PinState pin1(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_1);
  PinState pin2(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_2);
  EXPECT_NE(pin1, pin2);
}

TEST(PinStateTests, shouldBeEqual)
{
  PinState pin1(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_1);
  PinState pin2(EXPECTED_ID_1, EXPECTED_NAME_1, EXPECTED_STATE_1);
  EXPECT_EQ(pin1, pin2);
}

TEST(PinStateTests, shouldBePrintedCorrectly)
{
  PinState pin_state(5u, "pin_name", true);
  EXPECT_EQ(fmt::format("{}", pin_state), "PinState(id = 5, name = pin_name, state = true)");
}
}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
