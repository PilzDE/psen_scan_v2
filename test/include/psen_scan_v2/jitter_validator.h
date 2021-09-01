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

#ifndef PSEN_SCAN_V2_TEST_JITTER_VALIDATOR_H
#define PSEN_SCAN_V2_TEST_JITTER_VALIDATOR_H

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

#include <fmt/format.h>

#include "psen_scan_v2_standalone/util/logging.h"
#include "psen_scan_v2_standalone/util/timestamp.h"

using namespace psen_scan_v2_standalone;

namespace psen_scan_v2_test
{
template <typename ScanType>
int64_t getTimestamp(const ScanType& scan);

// Times are represented in nanoseconds if not otherwise indicated.
template <typename ScanType>
class JitterValidator
{
public:
  JitterValidator(unsigned int sample_size, int64_t period);
  void laserScanCallback(const ScanType& scan);
  bool waitForSaturation(int64_t wait_time_sec);
  ::testing::AssertionResult validateTimestamps(int64_t max_jitter) const;
  ::testing::AssertionResult validateCallbackInvocationTimes(int64_t max_jitter) const;

private:
  ::testing::AssertionResult validateVector(const std::vector<int64_t>& data, int64_t max_jitter) const;

private:
  unsigned int sample_size_;
  int64_t period_;
  bool saturated_{ false };
  std::mutex mutex_;
  std::condition_variable condition_variable_;
  std::vector<int64_t> scan_timestamps_;
  std::vector<int64_t> callback_invocation_times_;
};

template <typename ScanType>
JitterValidator<ScanType>::JitterValidator(unsigned int sample_size, int64_t period)
  : sample_size_(sample_size), period_(period)
{
  scan_timestamps_.reserve(sample_size_);
  callback_invocation_times_.reserve(sample_size_);
}

template <typename ScanType>
void JitterValidator<ScanType>::laserScanCallback(const ScanType& scan)
{
  if (!saturated_)
  {
    callback_invocation_times_.push_back(util::getCurrentTime());
    scan_timestamps_.push_back(getTimestamp(scan));

    if (callback_invocation_times_.size() == sample_size_)
    {
      saturated_ = true;
      condition_variable_.notify_all();
    }
  }
}

template <typename ScanType>
bool JitterValidator<ScanType>::waitForSaturation(int64_t wait_time_sec)
{
  std::unique_lock<std::mutex> lock(mutex_);
  return condition_variable_.wait_for(lock, std::chrono::seconds(wait_time_sec), [this] { return saturated_; });
}

template <typename ScanType>
::testing::AssertionResult JitterValidator<ScanType>::validateTimestamps(int64_t max_jitter) const
{
  return validateVector(scan_timestamps_, max_jitter) << " (timestamps)";
}

template <typename ScanType>
::testing::AssertionResult JitterValidator<ScanType>::validateCallbackInvocationTimes(int64_t max_jitter) const
{
  return validateVector(callback_invocation_times_, max_jitter) << " (callback invocation times)";
}

template <typename ScanType>
::testing::AssertionResult JitterValidator<ScanType>::validateVector(const std::vector<int64_t>& data,
                                                                     int64_t max_jitter) const
{
  unsigned int number_of_violations{ 0 };
  for (std::size_t i = 1; i < data.size(); ++i)
  {
    const auto diff{ data[i] - data[i - 1] };
    if (std::abs(diff - period_ > max_jitter))
    {
      number_of_violations++;
      PSENSCAN_WARN("JitterValidator",
                    "Found jitter of {}ms at time {} (scan number {}).",
                    (diff - period_) / 1000000.,
                    data[i],
                    i + 1);
    }
  }
  if (number_of_violations > 0)
  {
    return ::testing::AssertionFailure() << fmt::format("Found {}% violations of 1ms jitter criteria.",
                                                        100. * number_of_violations / sample_size_);
  }
  return ::testing::AssertionSuccess();
}

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_JITTER_VALIDATOR_H
