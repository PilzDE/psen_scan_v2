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
#include <vector>

#include "psen_scan_v2_standalone/util/format_range.h"

using namespace psen_scan_v2_standalone;

TEST(FormatRangeTest, stringFormat)
{
  EXPECT_EQ(util::formatRange<std::vector<std::string>>({ "a", "b", "c" }), "{a, b, c}");
}

TEST(FormatRangeTest, intFormat)
{
  EXPECT_EQ(util::formatRange<std::vector<int>>({ -1, 2, 3 }), "{-1, 2, 3}");
}

TEST(FormatRangeTest, doubleFormat)
{
  EXPECT_EQ(util::formatRange<std::vector<double>>({ -1.0, 2.0, 3.2 }), "{-1.0, 2.0, 3.2}");
}

TEST(FormatRangeTest, doubleFormat2Precision)
{
  EXPECT_EQ(util::formatRange<std::vector<double>>({ -1.01, 2.023, 3.237 }, 2), "{-1.01, 2.02, 3.24}");
}

TEST(FormatRangeTest, empty)
{
  EXPECT_EQ(util::formatRange<std::vector<double>>({}), "{}");
}

TEST(FormatRangeTest, single)
{
  EXPECT_EQ(util::formatRange<std::vector<int>>({ 1 }), "{1}");
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
