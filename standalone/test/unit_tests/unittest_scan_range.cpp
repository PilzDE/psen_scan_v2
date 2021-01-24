// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

#include <stdexcept>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/configuration/scan_range.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
static constexpr TenthOfDegree MIN_ALLOWED_ANGLE{ 7 };
static constexpr TenthOfDegree MAX_ALLOWED_ANGLE{ 54 };

static constexpr TenthOfDegree VALID_START_ANGLE{ MIN_ALLOWED_ANGLE.value() + 1 };
static constexpr TenthOfDegree VALID_END_ANGLE{ MAX_ALLOWED_ANGLE.value() - 1 };

static constexpr TenthOfDegree TOO_SMALL_SCAN_ANGLE{ MIN_ALLOWED_ANGLE.value() - 1 };
static constexpr TenthOfDegree TOO_LARGE_SCAN_ANGLE{ MAX_ALLOWED_ANGLE.value() + 1 };

using TestScanRange = configuration::ScanRange<MIN_ALLOWED_ANGLE.value(), MAX_ALLOWED_ANGLE.value()>;

TEST(ScanRangeTest, testCtorCallForCoverage)
{
  std::unique_ptr<configuration::DefaultScanRange> ptr{ new configuration::DefaultScanRange(MIN_ALLOWED_ANGLE,
                                                                                            MAX_ALLOWED_ANGLE) };
}

TEST(ScanRangeTest, testStartAngleTooSmall)
{
  EXPECT_THROW(TestScanRange(TOO_SMALL_SCAN_ANGLE, VALID_END_ANGLE), std::out_of_range);
}

TEST(ScanRangeTest, testStartAngleTooLarge)
{
  EXPECT_THROW(TestScanRange(TOO_LARGE_SCAN_ANGLE, VALID_END_ANGLE), std::out_of_range);
}

TEST(ScanRangeTest, testEndAngleTooSmall)
{
  EXPECT_THROW(TestScanRange(VALID_START_ANGLE, TOO_SMALL_SCAN_ANGLE), std::out_of_range);
}

TEST(ScanRangeTest, testEndAngleTooLarge)
{
  EXPECT_THROW(TestScanRange(VALID_START_ANGLE, TOO_LARGE_SCAN_ANGLE), std::out_of_range);
}

TEST(ScanRangeTest, testEndAngleSmallerThanStartAngle)
{
  EXPECT_THROW(TestScanRange(TenthOfDegree(17), TenthOfDegree(15)), std::invalid_argument);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
