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

#include <map>
#include <sstream>
#include <stdlib.h>
#include <string>

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>

#include "psen_scan_v2_standalone/core.h"

#include "psen_scan_v2/dist.h"
#include "psen_scan_v2/laserscan_validator.h"

using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_test;

namespace psen_scan_v2_standalone_test
{
static const char* TESTFILE_ENV_VAR{ "HW_TEST_SCAN_COMPARE_TESTFILE" };
static const TenthOfDegree ANGLE_START{ 687 };
static const TenthOfDegree ANGLE_END{ 2063 };
static const std::string HOST_IP{ "192.168.0.50" };
static const std::string SCANNER_IP{ "192.168.0.10" };
static int TEST_DURATION_S{ 10 };

class ScanComparisonTests : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    setLogLevel(CONSOLE_BRIDGE_LOG_INFO);
    PSENSCAN_INFO("ScanComparisonTests", "Using test duration={}", TEST_DURATION_S);

    const char* filepath{ std::getenv(TESTFILE_ENV_VAR) };
    if (!filepath)
    {
      PSENSCAN_ERROR("ScanComparisonTests", "Environment variable {} not set!", TESTFILE_ENV_VAR);
      FAIL();
    }
    PSENSCAN_INFO("ScanComparisonTests", "Using testfile {}", filepath);
    if (!boost::filesystem::exists(filepath))
    {
      PSENSCAN_ERROR("ScanComparisonTests", "File {} not found!", filepath);
      FAIL();
    }
    bins_expected_ = binsFromRosbag(filepath);
  }

protected:
  static std::map<int16_t, NormalDist> bins_expected_;
};

std::map<int16_t, NormalDist> ScanComparisonTests::bins_expected_{};

TEST_F(ScanComparisonTests, simpleCompare)
{
  size_t window_size = 120;  // Keep this high to avoid undersampling

  LaserScanValidator<LaserScan> laser_scan_validator(bins_expected_);
  laser_scan_validator.reset();

  DefaultScanRange scan_range{ ANGLE_START, ANGLE_END };

  ScannerConfigurationBuilder config_builder;
  config_builder.hostIP(HOST_IP).scannerIp(SCANNER_IP).scanRange(scan_range);

  ScannerV2 scanner(config_builder.build(), [&laser_scan_validator, &window_size](const LaserScan& scan) {
    return laser_scan_validator.scanCb(boost::make_shared<LaserScan const>(scan), window_size);
  });
  scanner.start();

  EXPECT_TRUE(laser_scan_validator.waitForResult(TEST_DURATION_S));

  scanner.stop();
}
}  // namespace psen_scan_v2_standalone_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  if (argc > 1)
  {
    std::istringstream is(argv[1]);
    int test_duration;
    if (is >> test_duration)
    {
      psen_scan_v2_standalone_test::TEST_DURATION_S = test_duration;
    }
  }
  return RUN_ALL_TESTS();
}
