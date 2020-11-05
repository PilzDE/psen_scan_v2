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
#ifndef PSEN_SCAN_V2_MESSAGE_COLLECTOR_H
#define PSEN_SCAN_V2_MESSAGE_COLLECTOR_H

#include <future>
#include <map>
#include <vector>
#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/dist.h"

#include <ros/ros.h>

namespace psen_scan_v2_test
{
void addScanToBin(const sensor_msgs::LaserScanConstPtr& scan, std::map<int16_t, NormalDist>& bin)
{
  if (scan == nullptr)
  {
    throw;
  }

  for (size_t i = 0; i < scan->ranges.size(); ++i)
  {
    auto bin_addr = psen_scan_v2::radToTenthDegree(scan->angle_min + scan->angle_increment * i);

    if (bin.find(bin_addr) == bin.end())
    {
      bin.emplace(bin_addr, NormalDist{});
      // Create bin
    }
    bin[bin_addr].update(scan->ranges[i]);
  }
}

std::map<int16_t, NormalDist> binsFromScans(std::vector<sensor_msgs::LaserScanConstPtr> scans)
{
  std::map<int16_t, NormalDist> bins;
  std::for_each(
      scans.cbegin(), scans.cend(), [&bins](const sensor_msgs::LaserScanConstPtr& scan) { addScanToBin(scan, bins); });
  return bins;
}

template <typename MsgType>
class MessageValidator
{
public:
  MessageValidator(ros::NodeHandle& nh, std::map<int16_t, NormalDist> bins_expected)
    : nh_(nh), bins_expected_(bins_expected){};

  typedef boost::shared_ptr<MsgType const> MsgTypeConstPtr;

  void scanCb(const MsgTypeConstPtr scan, size_t n_msgs)
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

        auto dist_actual = bin_actual.second;
        auto dist_expected = bin_expected->second;

        if (bin_expected == bins_expected_.end())
        {
          check_result_.set_value(::testing::AssertionFailure() << "Did not find expected value for angle "
                                                                << bin_actual.first / 10.
                                                                << " in the given reference scan\n");
          check_done_ = true;
        }
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

      if (counter_deviations > 0)
      {
        check_result_.set_value(::testing::AssertionFailure() << "\n" << error_string);
        check_done_ = true;
      }

      msgs_.clear();
    }
  }

  ::testing::AssertionResult validateMsgs(size_t n_msgs, std::string topic, const int duration)
  {
    msgs_.clear();

    check_result_ = std::promise<::testing::AssertionResult>();

    auto future = check_result_.get_future();
    sub_ = nh_.subscribe<MsgType>(
        topic, 1000, boost::bind(&MessageValidator::scanCb, this, boost::placeholders::_1, n_msgs));

    std::future_status status = future.wait_for(std::chrono::seconds(duration));

    // If the future timeouts no failure were detected, thus this means the test result is a success
    if (status == std::future_status::timeout)
    {
      check_done_ = true;
      return ::testing::AssertionSuccess();
    }

    return future.get();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  std::vector<MsgTypeConstPtr> msgs_;

  std::promise<::testing::AssertionResult> check_result_;

  std::map<int16_t, NormalDist> bins_expected_;

  std::atomic_bool check_done_{ false };
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_MESSAGE_COLLECTOR_H