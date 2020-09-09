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
#include "psen_scan_v2/build_ros_message_exception.h"
#include "psen_scan_v2/get_ros_parameter_exception.h"
#include "psen_scan_v2/psen_scan_v2_fatal_exception.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
TEST(BuildROSMessageExceptionTest, new_build_ros_message_exception)
{
  std::string except_str = "BuildROSMessageException";
  std::unique_ptr<BuildROSMessageException> e(new BuildROSMessageException(except_str));
  EXPECT_EQ(except_str, e->what());
}

TEST(GetROSParameterExceptionTest, new_param_missing_on_server_exception)
{
  std::string except_str = "GetROSParameterException";
  std::unique_ptr<ParamMissingOnServer> e(new ParamMissingOnServer(except_str));
  EXPECT_EQ(except_str, e->what());
}

TEST(GetROSParameterExceptionTest, new_wrong_parameter_type_exception)
{
  std::string except_str = "GetROSParameterException";
  std::unique_ptr<WrongParameterType> e(new WrongParameterType(except_str));
  EXPECT_EQ(except_str, e->what());
}

TEST(PSENScanV2FatalExceptionTest, new_psen_scan_v2_fatal_exception)
{
  std::string except_str = "PSENScanV2FatalException";
  std::unique_ptr<PSENScanV2FatalException> e(new PSENScanV2FatalException(except_str));
  EXPECT_EQ(except_str, e->what());
}
}  // namespace psen_scan_v2_test