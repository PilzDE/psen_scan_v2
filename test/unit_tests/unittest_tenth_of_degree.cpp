// Copyright (c) 2020 Pilz GmbH & Co. KG
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

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/tenth_of_degree.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
TEST(TenthOfDegreeTest, constructorTest)
{
  TenthOfDegree tenth_of_degree{ 1 };
}

TEST(TenthOfDegreeTest, valueTest)
{
  TenthOfDegree tenth_of_degree{ 1 };
  EXPECT_EQ(tenth_of_degree.value(), 1);
}

TEST(TenthOfDegreeTest, toRad)
{
  TenthOfDegree tenth_of_degree{ 1 };
  EXPECT_EQ(tenth_of_degree.toRad(), tenthDegreeToRad(1));
}

TEST(TenthOfDegreeTest, Multiplication)
{
  TenthOfDegree tenth_of_degree_1{ 2 };
  TenthOfDegree tenth_of_degree_2{ 3 };

  EXPECT_EQ((tenth_of_degree_1 * tenth_of_degree_2).value(), 6);
}

TEST(TenthOfDegreeTest, Addition)
{
  TenthOfDegree tenth_of_degree_1{ 2 };
  TenthOfDegree tenth_of_degree_2{ 3 };

  EXPECT_EQ((tenth_of_degree_1 + tenth_of_degree_2).value(), 5);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
