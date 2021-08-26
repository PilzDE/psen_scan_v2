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
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <functional>
#include <mutex>
#include <numeric>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/core.h"

using namespace psen_scan_v2_standalone;

static const char* HOST_IP_ENV_VAR{ "HOST_IP" };
static const char* SENSOR_IP_ENV_VAR{ "SENSOR_IP" };
static constexpr int HOST_UDP_PORT_DATA{ 55008 };
static constexpr std::size_t SAMPLE_SIZE{ 1000 };
static constexpr int WAIT_TIME_SEC{ 50 };

static int64_t getCurrentTime()
{
  return std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now())
      .time_since_epoch()
      .count();
}

class JitterValidator
{
public:
  JitterValidator()
  {
    scan_timestamps_.reserve(SAMPLE_SIZE);
    callback_invocation_times_.reserve(SAMPLE_SIZE);
  }

  void laserScanCallback(const LaserScan& scan)
  {
    if (!saturated_)
    {
      callback_invocation_times_.push_back(getCurrentTime());
      scan_timestamps_.push_back(scan.getTimestamp());

      if (scan_timestamps_.size() == SAMPLE_SIZE)
      {
        saturated_ = true;
        condition_variable_.notify_all();
      }
    }
  }

  bool waitForSaturation()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return condition_variable_.wait_for(lock, std::chrono::seconds(WAIT_TIME_SEC), [this] { return saturated_; });
  }

  ::testing::AssertionResult validateTimestamps() const
  {
    return validateVector(scan_timestamps_) << " (timestamps)";
  }

  ::testing::AssertionResult validateCallbackInvocationTimes() const
  {
    return validateVector(callback_invocation_times_) << " (callback invocation times)";
  }

private:
  ::testing::AssertionResult validateVector(const std::vector<int64_t>& data) const
  {
    std::vector<int64_t> diffs(SAMPLE_SIZE);
    std::adjacent_difference(data.begin(), data.end(), diffs.begin());
    const auto number_of_violations{ std::count_if(
        std::next(diffs.begin()), diffs.end(), [](int64_t diff) { return std::abs(diff - 30000000) > 1000000; }) };
    if (number_of_violations > 0)
    {
      PSENSCAN_WARN("JitterValidator", "Found {} violations of 1ms jitter criteria.", number_of_violations);
    }
    if (number_of_violations > 4)
    {
      return ::testing::AssertionFailure() << "Too many violations of 1ms jitter criteria";
    }
    return ::testing::AssertionSuccess();
  }

private:
  bool saturated_{ false };
  std::mutex mutex_;
  std::condition_variable condition_variable_;
  std::vector<int64_t> scan_timestamps_;
  std::vector<int64_t> callback_invocation_times_;
};

TEST(JitterStandaloneTests, testJitter)
{
  JitterValidator jitter_validator;

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

  ScannerV2 scanner(config_builder.build(),
                    std::bind(&JitterValidator::laserScanCallback, &jitter_validator, std::placeholders::_1));
  scanner.start();
  EXPECT_TRUE(jitter_validator.waitForSaturation()) << "Could not gather enough timing data from laserscan callbacks.";
  scanner.stop();
  EXPECT_TRUE(jitter_validator.validateTimestamps());
  EXPECT_TRUE(jitter_validator.validateCallbackInvocationTimes());
}

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
