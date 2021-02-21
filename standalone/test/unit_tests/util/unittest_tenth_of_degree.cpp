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

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_standalone_test
{
TEST(TenthOfDegreeTest, fromRad)
{
  const double angle_in_rad{ 0.1 };
  util::TenthOfDegree tenth_of_degree{ util::TenthOfDegree::fromRad(angle_in_rad) };
  EXPECT_EQ(tenth_of_degree.value(), data_conversion_layer::radToTenthDegree(angle_in_rad));
}

TEST(TenthOfDegreeTest, valueTest)
{
  util::TenthOfDegree tenth_of_degree{ 1 };
  EXPECT_EQ(tenth_of_degree.value(), 1);
}

TEST(TenthOfDegreeTest, negativeValueTest)
{
  util::TenthOfDegree tenth_of_degree{ -1 };
  EXPECT_EQ(tenth_of_degree.value(), -1);
}

TEST(TenthOfDegreeTest, toRad)
{
  util::TenthOfDegree tenth_of_degree{ 1 };
  EXPECT_DOUBLE_EQ(tenth_of_degree.toRad(), data_conversion_layer::tenthDegreeToRad(1));
}

TEST(TenthOfDegreeTest, Multiplication)
{
  util::TenthOfDegree tenth_of_degree_1{ 2 };
  util::TenthOfDegree tenth_of_degree_2{ 3 };

  EXPECT_EQ((tenth_of_degree_1 * tenth_of_degree_2).value(), 6);
}

TEST(TenthOfDegreeTest, MultiplicationWithInteger)
{
  util::TenthOfDegree tenth_of_degree{ 2 };
  const int int_value{ 3 };

  EXPECT_EQ((tenth_of_degree * int_value).value(), 6);
}

TEST(TenthOfDegreeTest, DivisionWithInteger)
{
  util::TenthOfDegree tenth_of_degree{ 6 };
  const int int_value{ 3 };

  EXPECT_EQ((tenth_of_degree / int_value).value(), 2);
}

TEST(TenthOfDegreeTest, Addition)
{
  util::TenthOfDegree tenth_of_degree_1{ 2 };
  util::TenthOfDegree tenth_of_degree_2{ 3 };

  EXPECT_EQ((tenth_of_degree_1 + tenth_of_degree_2).value(), 5);
}

TEST(TenthOfDegreeTest, Subtraction)
{
  util::TenthOfDegree tenth_of_degree_1{ 2 };
  util::TenthOfDegree tenth_of_degree_2{ 3 };

  EXPECT_EQ((tenth_of_degree_1 - tenth_of_degree_2).value(), -1);
}

TEST(TenthOfDegreeTest, EqualityComparison)
{
  EXPECT_TRUE(util::TenthOfDegree(1) == util::TenthOfDegree(1));
  EXPECT_FALSE(util::TenthOfDegree(1) == util::TenthOfDegree(2));
}

TEST(TenthOfDegreeTest, LessThanComparison)
{
  EXPECT_TRUE(util::TenthOfDegree(1) < util::TenthOfDegree(2));
  EXPECT_FALSE(util::TenthOfDegree(1) < util::TenthOfDegree(1));
}

TEST(TenthOfDegreeTest, LargerThanComparison)
{
  EXPECT_TRUE(util::TenthOfDegree(2) > util::TenthOfDegree(1));
  EXPECT_FALSE(util::TenthOfDegree(1) > util::TenthOfDegree(1));
}

TEST(TenthOfDegreeTest, LessThanOrEqualComparison)
{
  EXPECT_TRUE(util::TenthOfDegree(1) <= util::TenthOfDegree(2));
  EXPECT_TRUE(util::TenthOfDegree(1) <= util::TenthOfDegree(1));
  EXPECT_FALSE(util::TenthOfDegree(2) <= util::TenthOfDegree(1));
}

TEST(TenthOfDegreeTest, LargerThanOrEqualComparison)
{
  EXPECT_TRUE(util::TenthOfDegree(2) >= util::TenthOfDegree(1));
  EXPECT_TRUE(util::TenthOfDegree(1) >= util::TenthOfDegree(1));
  EXPECT_FALSE(util::TenthOfDegree(1) >= util::TenthOfDegree(2));
}

TEST(TenthOfDegreeTest, shouldThrowUnderflowErrorOnCastToIntOfValuesUnderZero)
{
  EXPECT_THROW(std::cout << static_cast<uint16_t>(util::TenthOfDegree(-1))
                         << " - printed to not be unused. Should throw before!\n",
               std::underflow_error);
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
