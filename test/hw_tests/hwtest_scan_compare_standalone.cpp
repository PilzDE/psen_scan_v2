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
static const char* HOST_IP_ENV_VAR{ "HOST_IP" };
static const char* SENSOR_IP_ENV_VAR{ "SENSOR_IP" };
static const util::TenthOfDegree ANGLE_START{ 687 };
static const util::TenthOfDegree ANGLE_END{ 2063 };
static constexpr int HOST_UDP_PORT_DATA{ 55006 };
static int TEST_DURATION_S{ 10 };

class ScanComparisonTests : public ::testing::Test
{
public:
  void SetUp() override  // Omit using SetUpTestSuite() for googletest below v1.11.0, see
                         // https://github.com/google/googletest/issues/247
  {
    setLogLevel(CONSOLE_BRIDGE_LOG_INFO);
    PSENSCAN_INFO("ScanComparisonTests", "Using test duration={}", TEST_DURATION_S);

    const char* host_ip{ std::getenv(HOST_IP_ENV_VAR) };
    if (host_ip)
    {
      host_ip_ = host_ip;
    }
    const char* scanner_ip{ std::getenv(SENSOR_IP_ENV_VAR) };
    if (scanner_ip)
    {
      scanner_ip_ = scanner_ip;
    }

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
  std::map<int16_t, NormalDist> bins_expected_{};
  std::string host_ip_{ "192.168.0.50" };
  std::string scanner_ip_{ "192.168.0.10" };
};

typedef psen_scan_v2_standalone::LaserScan ScanType;

TEST_F(ScanComparisonTests, simpleCompare)
{
  size_t window_size = 120;  // Keep this high to avoid undersampling

  LaserScanValidator<ScanType> laser_scan_validator(bins_expected_);
  laser_scan_validator.reset();

  ScanRange scan_range{ ANGLE_START, ANGLE_END };

  ScannerConfigurationBuilder config_builder;
  config_builder.hostIP(host_ip_).scannerIp(scanner_ip_).hostDataPort(HOST_UDP_PORT_DATA).scanRange(scan_range);

  ScannerV2 scanner(config_builder.build(), [&laser_scan_validator, &window_size](const ScanType& scan) {
    return laser_scan_validator.scanCb<-1375>(boost::make_shared<ScanType const>(scan), window_size);
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
