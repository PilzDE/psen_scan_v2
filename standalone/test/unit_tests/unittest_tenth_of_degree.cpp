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

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/angle_conversions.h"
#include "psen_scan_v2_standalone/tenth_of_degree.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
TEST(TenthOfDegreeTest, valueTest)
{
  TenthOfDegree tenth_of_degree{ 1 };
  EXPECT_EQ(tenth_of_degree.value(), 1);
}

TEST(TenthOfDegreeTest, negativeValueTest)
{
  TenthOfDegree tenth_of_degree{ -1 };
  EXPECT_EQ(tenth_of_degree.value(), -1);
}

TEST(TenthOfDegreeTest, toRad)
{
  TenthOfDegree tenth_of_degree{ 1 };
  EXPECT_DOUBLE_EQ(tenth_of_degree.toRad(), tenthDegreeToRad(1));
}

TEST(TenthOfDegreeTest, Multiplication)
{
  TenthOfDegree tenth_of_degree_1{ 2 };
  TenthOfDegree tenth_of_degree_2{ 3 };

  EXPECT_EQ((tenth_of_degree_1 * tenth_of_degree_2).value(), 6);
}

TEST(TenthOfDegreeTest, MultiplicationWithInteger)
{
  TenthOfDegree tenth_of_degree{ 2 };
  const int int_value{ 3 };

  EXPECT_EQ((tenth_of_degree * int_value).value(), 6);
}

TEST(TenthOfDegreeTest, Addition)
{
  TenthOfDegree tenth_of_degree_1{ 2 };
  TenthOfDegree tenth_of_degree_2{ 3 };

  EXPECT_EQ((tenth_of_degree_1 + tenth_of_degree_2).value(), 5);
}

TEST(TenthOfDegreeTest, EqualityComparison)
{
  EXPECT_TRUE(TenthOfDegree(1) == TenthOfDegree(1));
  EXPECT_FALSE(TenthOfDegree(1) == TenthOfDegree(2));
}

TEST(TenthOfDegreeTest, LessThanComparison)
{
  EXPECT_TRUE(TenthOfDegree(1) < TenthOfDegree(2));
  EXPECT_FALSE(TenthOfDegree(1) < TenthOfDegree(1));
}

TEST(TenthOfDegreeTest, LargerThanComparison)
{
  EXPECT_TRUE(TenthOfDegree(2) > TenthOfDegree(1));
  EXPECT_FALSE(TenthOfDegree(1) > TenthOfDegree(1));
}

TEST(TenthOfDegreeTest, LessThanOrEqualComparison)
{
  EXPECT_TRUE(TenthOfDegree(1) <= TenthOfDegree(2));
  EXPECT_TRUE(TenthOfDegree(1) <= TenthOfDegree(1));
  EXPECT_FALSE(TenthOfDegree(2) <= TenthOfDegree(1));
}

TEST(TenthOfDegreeTest, LargerThanOrEqualComparison)
{
  EXPECT_TRUE(TenthOfDegree(2) >= TenthOfDegree(1));
  EXPECT_TRUE(TenthOfDegree(1) >= TenthOfDegree(1));
  EXPECT_FALSE(TenthOfDegree(1) >= TenthOfDegree(2));
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
