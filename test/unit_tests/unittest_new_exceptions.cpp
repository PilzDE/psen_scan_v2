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
#include "psen_scan_v2/get_ros_parameter_exception.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
TEST(ParameterNotSetTests, whatShouldReturnExpectedString)
{
  std::string except_str = "GetROSParameterException";
  std::unique_ptr<ParameterNotSet> e(new ParameterNotSet(except_str));  // keep new to increase function coverage
  EXPECT_EQ(except_str, e->what());
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
