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

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/core.h"

#include "psen_scan_v2/test_data.h"
#include "psen_scan_v2/test_data_helper.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_test
{
static const std::string SCANNER_IP{ "192.168.0.10" };
static const int HOST_UDP_DATA_PORT{ 55007 };
static const util::TenthOfDegree ANGLE_START{ 1 };
static const util::TenthOfDegree ANGLE_END{ 2749 };

static constexpr int64_t SCANNER_RUN_DURATION_S{ 30 };
static constexpr std::size_t MINIMUM_TEST_SIZE{ 900 };
static constexpr int64_t TIME_COMPARISON_EPSILON_NS{ 1000000 };
static const std::string UDP_DATA_FILENAME_ENV_VAR{ "UDP_DATA_FILENAME" };

class TimestampTests : public ::testing::Test
{
public:
  void SetUp() override
  {
    const char* udp_data_filename{ std::getenv(UDP_DATA_FILENAME_ENV_VAR.c_str()) };
    ASSERT_NE(udp_data_filename, nullptr)
        << "Searching environment variable " << UDP_DATA_FILENAME_ENV_VAR << " failed.";
    udp_data_filename_ = udp_data_filename;

    buildScannerConfig();
    setLogLevel(CONSOLE_BRIDGE_LOG_INFO);

    assembleTestDataByRunningTheScanner();  // Only runs the scanner once for all test cases.
    ASSERT_GE(testSize(), MINIMUM_TEST_SIZE) << "Assembling test data failed.";
  }

protected:
  void assembleTestDataByRunningTheScanner()
  {
    // We use a static variable in order to avoid multiple setups with data assembly.
    // If not for https://github.com/google/googletest/issues/247 this could be done via SetUpTestSuite().
    static const std::shared_ptr<test_data::TestData> TEST_DATA_PTR{ test_data::assemble(
        *scanner_config_, SCANNER_RUN_DURATION_S, udp_data_filename_, HOST_UDP_DATA_PORT) };
    test_data_ptr_ = TEST_DATA_PTR;
  }

  const test_data::TestData& testData() const
  {
    return *test_data_ptr_;
  }

  std::size_t testSize() const
  {
    return test_data_ptr_->size();
  }

private:
  void buildScannerConfig()
  {
    ScannerConfigurationBuilder config_builder;
    config_builder.scannerIp(SCANNER_IP)
        .hostDataPort(HOST_UDP_DATA_PORT)
        .scanRange(ScanRange{ ANGLE_START, ANGLE_END });

    if (const char* host_ip{ std::getenv("HOST_IP") })
    {
      config_builder.hostIP(host_ip);
    }

    if (const char* scanner_ip{ std::getenv("SENSOR_IP") })
    {
      config_builder.scannerIp(scanner_ip);
    }

    scanner_config_.reset(new ScannerConfiguration(config_builder.build()));
  }

private:
  std::shared_ptr<test_data::TestData> test_data_ptr_;
  std::unique_ptr<ScannerConfiguration> scanner_config_;
  std::string udp_data_filename_;
};

TEST_F(TimestampTests, testTimestampIncreasing)
{
  for (std::size_t i = 1; i < testSize(); ++i)
  {
    EXPECT_GT(testData().at(i).timestamp(), testData().at(i - 1).timestamp())
        << "Timestamp not increasing for scan counter " << testData().at(i).scanCounter();
  }
}

TEST_F(TimestampTests, testTimestampIsNearOrGreaterThanLastUdpFrameTime)
{
  for (std::size_t i = 1; i < testSize(); ++i)
  {
    const auto& datum1{ testData().at(i) };
    const auto& datum2{ testData().at(i - 1) };
    EXPECT_GT(datum1.timestamp() - datum2.lastFrameTime(), -TIME_COMPARISON_EPSILON_NS)
        << "Timestamp " << datum1.timestamp() << " for scan counter " << datum1.scanCounter()
        << " not near or greater than last udp frame time " << datum2.lastFrameTime() << " for scan counter "
        << datum2.scanCounter();
  }
}

TEST_F(TimestampTests, testTimestampIsLessThanFirstUdpFrameTime)
{
  for (const auto& datum : testData())
  {
    EXPECT_LT(datum.timestamp(), datum.firstFrameTime())
        << "Timestamp not less then first udp frame time for scan counter " << datum.scanCounter();
  }
}

TEST_F(TimestampTests, testTimestampIsLessThenCallbackInvocationTime)
{
  for (const auto& datum : testData())
  {
    EXPECT_LT(datum.timestamp(), datum.callbackInvocationTime())
        << "Timestamp not less then callback invocation time for scan counter " << datum.scanCounter();
  }
}
}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
