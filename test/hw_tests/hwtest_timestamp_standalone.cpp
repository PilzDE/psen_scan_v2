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
#include "psen_scan_v2/test_data_assembler.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_test
{
static const std::string SCANNER_IP{ "192.168.0.10" };
static const int HOST_UDP_DATA_PORT{ 55007 };
static const util::TenthOfDegree ANGLE_START{ 1 };
static const util::TenthOfDegree ANGLE_END{ 2749 };

static constexpr int64_t SCANNER_RUN_DURATION_S{ 30 };
static constexpr std::size_t MINIMUM_TEST_SIZE{ 900 };
static const std::string UDP_DATA_FILENAME_ENV_VAR{ "UDP_DATA_FILENAME" };

double mean(const std::vector<int64_t>& vec)
{
  if (vec.empty())
  {
    throw std::invalid_argument("mean(): Vector must not be empty.");
  }
  const auto sum{ std::accumulate(vec.begin(), vec.end(), int64_t(0)) };
  return static_cast<double>(sum) / static_cast<double>(vec.size());
}

std::vector<int64_t> computeTimestampToFirstFrameTimeDiffs(const TestData& test_data)
{
  std::vector<int64_t> diff;
  diff.reserve(test_data.size());
  std::transform(test_data.begin(), test_data.end(), std::back_inserter(diff), [](const auto& datum) {
    return datum.timestamp() - datum.firstFrameTime();
  });
  return diff;
}

class TimestampTests : public ::testing::Test
{
public:
  void SetUp() override
  {
    const char* udp_data_filename{ std::getenv(UDP_DATA_FILENAME_ENV_VAR.c_str()) };
    ASSERT_NE(udp_data_filename, nullptr)
        << "Searching environment variable " << UDP_DATA_FILENAME_ENV_VAR << "failed.";
    udp_data_filename_ = udp_data_filename;

    buildScannerConfig();
    setLogLevel(CONSOLE_BRIDGE_LOG_INFO);
    ASSERT_GE(testSize(), MINIMUM_TEST_SIZE) << "Assembling test data failed.";
  }

protected:
  const TestData& testData() const
  {
    static const std::unique_ptr<TestData> test_data_ptr{ TestDataAssembler::assemble(
        *scanner_config_, SCANNER_RUN_DURATION_S, udp_data_filename_, HOST_UDP_DATA_PORT) };
    return *test_data_ptr;
  }

  std::size_t testSize() const
  {
    return testData().size();
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
  std::unique_ptr<ScannerConfiguration> scanner_config_;
  std::string udp_data_filename_;
};

TEST_F(TimestampTests, testTimestampIncreasing)
{
  for (std::size_t i = 1; i < testSize(); ++i)
  {
    EXPECT_GT(testData().at(i).timestamp(), testData().at(i - 1).timestamp());
  }
}

TEST_F(TimestampTests, testTimestampIsGreaterThanLastUdpFrameTime)
{
  for (std::size_t i = 1; i < testSize(); ++i)
  {
    EXPECT_GT(testData().at(i).timestamp(), testData().at(i - 1).lastFrameTime());
  }
}

TEST_F(TimestampTests, testTimestampIsLessThanFirstUdpFrameTime)
{
  for (const auto& datum : testData())
  {
    EXPECT_LT(datum.timestamp(), datum.firstFrameTime());
  }
}

TEST_F(TimestampTests, testTimestampIsLessThenCallbackInvocationTime)
{
  for (const auto& datum : testData())
  {
    EXPECT_LT(datum.timestamp(), datum.callbackInvocationTime());
  }
}

TEST_F(TimestampTests, testTimestampRelativeToFirstFrameTimeJitterIsLessThenOneMilliSec)
{
  const auto diffs{ computeTimestampToFirstFrameTimeDiffs(testData()) };
  double mean_value{ 0.0 };
  ASSERT_NO_THROW(mean_value = mean(diffs));
  for (const auto& diff : diffs)
  {
    EXPECT_LT(std::abs(static_cast<double>(diff) - mean_value), 1000000.0)
        << "Detected a jitter of timestamp relativ to first frame time above 1ms where the mean is " << mean_value;
  }
}
}  // namespace psen_scan_v2_test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
