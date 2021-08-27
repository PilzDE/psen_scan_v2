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

#include <cstdlib>
#include <functional>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/core.h"

#include "psen_scan_v2/jitter_validator.h"

using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_test;

static const char* HOST_IP_ENV_VAR{ "HOST_IP" };
static const char* SENSOR_IP_ENV_VAR{ "SENSOR_IP" };
static constexpr int HOST_UDP_PORT_DATA{ 55008 };
static constexpr std::size_t SAMPLE_SIZE{ 1000 };
static constexpr int WAIT_TIME_SEC{ 50 };
static constexpr int64_t MAX_JITTER_NSEC{ 1000000 };
static constexpr int64_t SCAN_PERIOD_NSEC{ 30000000 };
static const double MAX_OUTLIER_RATIO{ 0.002 };

static ScannerConfiguration setUpScannerConfiguration()
{
  const char* host_ip{ std::getenv(HOST_IP_ENV_VAR) };
  if (!host_ip)
  {
    host_ip = "auto";
  }
  const char* scanner_ip{ std::getenv(SENSOR_IP_ENV_VAR) };
  if (!scanner_ip)
  {
    scanner_ip = "192.168.0.10";
  }
  ScanRange scan_range{ util::TenthOfDegree{ 1 }, util::TenthOfDegree{ 2749 } };
  ScannerConfigurationBuilder config_builder;
  config_builder.hostIP(host_ip).scannerIp(scanner_ip).hostDataPort(HOST_UDP_PORT_DATA).scanRange(scan_range);
  return config_builder.build();
}

template <>
int64_t psen_scan_v2_test::getTimestamp<LaserScan>(const LaserScan& scan)
{
  return scan.getTimestamp();
}

TEST(JitterStandaloneTests, testJitterIsBelowOneMillisecond)
{
  JitterValidator<LaserScan> jitter_validator(SAMPLE_SIZE, SCAN_PERIOD_NSEC);
  ScannerV2 scanner(
      setUpScannerConfiguration(),
      std::bind(&JitterValidator<LaserScan>::laserScanCallback, &jitter_validator, std::placeholders::_1));

  scanner.start();
  EXPECT_TRUE(jitter_validator.waitForSaturation(WAIT_TIME_SEC));
  scanner.stop();

  EXPECT_TRUE(jitter_validator.validateTimestamps(MAX_JITTER_NSEC, MAX_OUTLIER_RATIO));
  EXPECT_TRUE(jitter_validator.validateCallbackInvocationTimes(MAX_JITTER_NSEC, MAX_OUTLIER_RATIO));
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
