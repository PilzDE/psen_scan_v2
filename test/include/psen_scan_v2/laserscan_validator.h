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
#ifndef PSEN_SCAN_V2_LASERSCAN_VALIDATOR_H
#define PSEN_SCAN_V2_LASERSCAN_VALIDATOR_H

#include <future>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>

// #include <ros/ros.h>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/laserscan.h"

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"

#include "psen_scan_v2/dist.h"

namespace psen_scan_v2_test
{
void addScanToBin(const psen_scan_v2_standalone::LaserScan& scan, std::map<int16_t, NormalDist>& bin)
{
  for (size_t i = 0; i < scan.getMeasurements().size(); ++i)
  {
    auto bin_addr = psen_scan_v2_standalone::data_conversion_layer::radToTenthDegree(scan.getMinScanAngle() +
                                                                                     scan.getScanResolution() * i);

    if (bin.find(bin_addr) == bin.end())
    {
      bin.emplace(bin_addr, NormalDist{});
      // Create bin
    }
    bin[bin_addr].update(scan.getMeasurements()[i]);
  }
}

void addScanToBin(const sensor_msgs::LaserScan& scan, std::map<int16_t, NormalDist>& bin)
{
  for (size_t i = 0; i < scan.ranges.size(); ++i)
  {
    auto bin_addr =
        psen_scan_v2_standalone::data_conversion_layer::radToTenthDegree(scan->angle_min + scan->angle_increment * i);

    if (bin.find(bin_addr) == bin.end())
    {
      bin.emplace(bin_addr, NormalDist{});
      // Create bin
    }
    bin[bin_addr].update(scan.ranges[i]);
  }
}

template <typename ScanConstPtr>
std::map<int16_t, NormalDist> binsFromScans(const std::vector<ScanConstPtr>& scans)
{
  std::map<int16_t, NormalDist> bins;
  std::for_each(scans.cbegin(), scans.cend(), [&bins](const ScanConstPtr& scan) {
    if (scan == nullptr)
    {
      throw std::invalid_argument("LaserScan pointer must not be null");
    }
    addScanToBin(*scan, bins);
  });
  return bins;
}

template <typename ScanType>
class LaserScanValidator
{
public:
  LaserScanValidator(std::map<int16_t, NormalDist> bins_expected) : bins_expected_(bins_expected){};

  typedef boost::shared_ptr<ScanType const> ScanConstPtr;

  void scanCb(const ScanConstPtr scan, size_t n_msgs)
  {
    ROS_INFO_THROTTLE(5, "Checking messages for validity. So far looking good.");

    if (check_done_)
    {
      return;
    }

    msgs_.push_back(scan);

    // To have only one call on the promise the subscriber is shut down
    if (msgs_.size() == n_msgs)
    {
      auto bins_actual = binsFromScans(msgs_);

      std::string error_string;
      std::size_t counter_deviations{ 0 };

      // Compare
      for (const auto& bin_actual : bins_actual)
      {
        auto bin_expected = bins_expected_.find(bin_actual.first);

        if (bin_expected == bins_expected_.end())
        {
          check_result_.set_value(::testing::AssertionFailure()
                                  << "Did not find expected value for angle " << bin_actual.first / 10.
                                  << " in the given reference scan\n");
          check_done_ = true;
          return;
        }

        auto dist_actual = bin_actual.second;
        auto dist_expected = bin_expected->second;
        auto distance = bhattacharyya_distance(dist_actual, dist_expected);
        if (distance > 10.)
        {
          error_string +=
              fmt::format("On {:+.1f} deg  expected: {} actual: {} | dist: {:.1f}, dmean: {:.3f}, dstdev: {:.3f}\n",
                          bin_actual.first / 10.,
                          dist_expected,
                          dist_actual,
                          distance,
                          abs(dist_expected.mean() - dist_actual.mean()),
                          abs(dist_expected.stdev() - dist_actual.stdev()));
          counter_deviations++;
        }
      }

      number_of_comparisons_++;

      if (counter_deviations > 0)
      {
        check_result_.set_value(::testing::AssertionFailure() << "\n" << error_string);
        check_done_ = true;
      }

      msgs_.clear();
    }
  }

  void reset()
  {
    msgs_.clear();
    check_result_ = std::promise<::testing::AssertionResult>();
    check_result_future_ = check_result_.get_future();
  }

  ::testing::AssertionResult waitForResult(const int duration)
  {
    std::future_status status = check_result_future_.wait_for(std::chrono::seconds(duration));

    // If the future timeouts no failure were detected, thus this means the test result is a success
    if (status == std::future_status::timeout)
    {
      check_done_ = true;
      if (number_of_comparisons_ == 0)
      {
        return ::testing::AssertionFailure() << "Did not perform any checks. Check if laserscans are published.";
      }
      return ::testing::AssertionSuccess();
    }

    return check_result_future_.get();
  }

private:
  std::vector<ScanConstPtr> msgs_;

  std::promise<::testing::AssertionResult> check_result_;

  std::future<::testing::AssertionResult> check_result_future_;

  std::map<int16_t, NormalDist> bins_expected_;

  std::atomic_bool check_done_{ false };

  std::atomic_ulong number_of_comparisons_{ 0UL };
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_LASERSCAN_VALIDATOR_H
