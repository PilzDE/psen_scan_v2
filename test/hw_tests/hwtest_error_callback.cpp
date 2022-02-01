// Copyright (c) 2022 Pilz GmbH & Co. KG
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

#include <chrono>
#include <string>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scanner_config_builder.h"
#include "psen_scan_v2_standalone/scan_range.h"

#include "psen_scan_v2/default_ros_parameters.h"
#include "psen_scan_v2/ros_scanner_node.h"

using namespace ::testing;
using namespace std::chrono_literals;
using namespace psen_scan_v2;
namespace standalone = psen_scan_v2_standalone;

namespace psen_scan_v2_test
{
class ErrorCallbackTests : public Test
{
};

TEST_F(ErrorCallbackTests, shouldCallErrorCallbackWhenScannerIsTurnedOffToLong)
{
  ros::NodeHandle pnh("~");

  const util::TenthOfDegree ANGLE_START{ data_conversion_layer::degreeToTenthDegree(137) };
  const util::TenthOfDegree ANGLE_END{ data_conversion_layer::degreeToTenthDegree(138) };
  const int WRONG_DATA_PORT = 9999;

  ScannerConfiguration scanner_configuration{ ScannerConfigurationBuilder("192.168.0.10")
                                                  .scanRange(ScanRange{ ANGLE_START, ANGLE_END })
                                                  .hostDataPort(WRONG_DATA_PORT)
                                                  .secondsUntilDataTimeoutCountsAsError(2.0)
                                                  .build() };

  ROSScannerNode ros_scanner_node(pnh, "test/scan", "test", DEFAULT_X_AXIS_ROTATION, scanner_configuration);

  EXPECT_NO_BLOCK_WITH_TIMEOUT_AND_NO_THROW(ros_scanner_node.run();, 5s);
}
}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "hwtest_error_callback");
  return RUN_ALL_TESTS();
}
