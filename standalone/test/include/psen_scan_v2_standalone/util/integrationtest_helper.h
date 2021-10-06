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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_INTEGRATIONTEST_HELPER_H
#define PSEN_SCAN_V2_STANDALONE_TEST_INTEGRATIONTEST_HELPER_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/configuration/scanner_ids.h"
#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/data_conversion_layer/laserscan_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"
#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/scan_range.h"
#include "psen_scan_v2_standalone/util/timestamp.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;

#define EXPECT_FUTURE_IS_READY(future) EXPECT_EQ(future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready)

#define EXPECT_FUTURE_TIMEOUT(future, wait_timeout)                                                                    \
  EXPECT_EQ(future.wait_for(wait_timeout), std::future_status::timeout)

#define EXPECT_BARRIER_OPENS(barrier, wait_timeout) EXPECT_TRUE(barrier.waitTillRelease(wait_timeout))

#define EXPECT_DOES_NOT_BLOCK(statement)                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    const auto future = std::async(std::launch::async, [&]() { statement });                                           \
    EXPECT_FUTURE_IS_READY(future) << #statement << " does not return.";                                               \
  } while (false)  // https://stackoverflow.com/questions/1067226/c-multi-line-macro-do-while0-vs-scope-block

#define EXPECT_STOP_REQUEST_CALL(hw_mock)                                                                              \
  EXPECT_CALL(hw_mock, receiveControlMsg(_, data_conversion_layer::stop_request::serialize()))

#define EXPECT_START_REQUEST_CALL(hw_mock, config)                                                                     \
  EXPECT_CALL(                                                                                                         \
      hw_mock,                                                                                                         \
      receiveControlMsg(                                                                                               \
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config))))

#define EXPECT_CALLBACK_WILL_OPEN_BARRIER(cb, msgs, barrier)                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    const auto timestamp{ util::getCurrentTime() };                                                                    \
    const auto scan{ createReferenceScan(msgs, timestamp) };                                                           \
    EXPECT_CALL(cb, LaserScanCallback(AllOf(ScanDataEqual(scan), TimestampInExpectedTimeframe(scan, timestamp))))      \
        .WillOnce(OpenBarrier(&barrier));                                                                              \
  } while (false)

#define EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock, driver, config)                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    util::Barrier start_req_barrier;                                                                                   \
    std::future<void> start_future;                                                                                    \
    EXPECT_START_REQUEST_CALL(*hw_mock, *config).WillOnce(OpenBarrier(&start_req_barrier));                            \
    EXPECT_DOES_NOT_BLOCK(start_future = driver->start(););                                                            \
    EXPECT_BARRIER_OPENS(start_req_barrier, DEFAULT_TIMEOUT) << "Start request not received";                          \
    hw_mock->sendStartReply();                                                                                         \
    EXPECT_FUTURE_IS_READY(start_future) << "Scanner::start() not finished";                                           \
  } while (false)

#define EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock, driver)                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    util::Barrier stop_req_barrier;                                                                                    \
    std::future<void> stop_future;                                                                                     \
    EXPECT_STOP_REQUEST_CALL(*hw_mock).WillOnce(OpenBarrier(&stop_req_barrier));                                       \
    EXPECT_DOES_NOT_BLOCK(stop_future = driver->stop(););                                                              \
    EXPECT_BARRIER_OPENS(stop_req_barrier, DEFAULT_TIMEOUT) << "Stop request not received";                            \
    hw_mock->sendStopReply();                                                                                          \
    EXPECT_FUTURE_IS_READY(stop_future) << "Scanner::stop() not finished";                                             \
  } while (false)

static constexpr ScanRange DEFAULT_SCAN_RANGE{ util::TenthOfDegree(1), util::TenthOfDegree(60) };
static constexpr util::TenthOfDegree DEFAULT_SCAN_RESOLUTION{ 2 };
static constexpr int64_t DEFAULT_TIMESTAMP{ 1000000000 };

static double randDouble(double low, double high)
{
  static std::default_random_engine re{};
  using Dist = std::uniform_real_distribution<double>;
  static Dist uid{};
  return uid(re, Dist::param_type{ low, high });
}

static double restrictToOneDigitsAfterComma(const double& value)
{
  return std::round(value * 10.) / 10.;
}

static std::vector<double> generateMeasurements(const unsigned int& num_elements, const double& low, const double& high)
{
  std::vector<double> vec(num_elements);
  // The scanner sends tenth degree values. Therefore, restrict values to one digit after the comma.
  std::generate(vec.begin(), vec.end(), [low, high]() { return restrictToOneDigitsAfterComma(randDouble(low, high)); });
  return vec;
}

static std::vector<double> generateIntensities(const unsigned int& num_elements, const double& low, const double& high)
{
  std::vector<double> vec(num_elements);
  // The scanner sends intensities as int values, therefore, the values are rounded.
  std::generate(vec.begin(), vec.end(), [low, high]() { return std::round(randDouble(low, high)); });
  return vec;
}

static data_conversion_layer::monitoring_frame::Message
createValidMonitoringFrameMsg(const uint32_t scan_counter = 42,
                              const util::TenthOfDegree start_angle = DEFAULT_SCAN_RANGE.getStart(),
                              const util::TenthOfDegree end_angle = DEFAULT_SCAN_RANGE.getEnd())
{
  const auto resolution{ util::TenthOfDegree(10) };

  const unsigned int num_elements = ((end_angle - start_angle) / resolution).value();
  const double lowest_measurement{ 0. };
  const double highest_measurement{ 10. };
  const std::vector<double> measurements{ generateMeasurements(num_elements, lowest_measurement, highest_measurement) };

  const double lowest_intensity{ 0. };
  const double highest_intensity{ 16383. };  // only 14 of 16 bits can be used for the actual intensity value
  const std::vector<double> intensities{ generateIntensities(num_elements, lowest_intensity, highest_intensity) };

  const std::vector<data_conversion_layer::monitoring_frame::diagnostic::Message> diagnostic_messages{
    { configuration::ScannerId::master, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(1, 7) }
  };

  return data_conversion_layer::monitoring_frame::Message(
      start_angle, resolution, scan_counter, measurements, intensities, diagnostic_messages);
}

std::vector<data_conversion_layer::monitoring_frame::Message>
createValidMonitoringFrameMsgs(const uint32_t scan_counter, const std::size_t num_elements)
{
  std::vector<data_conversion_layer::monitoring_frame::Message> msgs;
  std::generate_n(
      std::back_inserter(msgs), num_elements, [scan_counter]() { return createValidMonitoringFrameMsg(scan_counter); });
  return msgs;
}

std::vector<data_conversion_layer::monitoring_frame::Message>
createMonitoringFrameMsgsForScanRound(const uint32_t scan_counter, const std::size_t num_elements)
{
  std::vector<data_conversion_layer::monitoring_frame::Message> msgs;
  for (std::size_t i = 0; i < num_elements; ++i)
  {
    const util::TenthOfDegree start_angle =
        (DEFAULT_SCAN_RANGE.getEnd() / static_cast<int>(num_elements)) * static_cast<int>(i);
    const util::TenthOfDegree end_angle =
        (DEFAULT_SCAN_RANGE.getEnd() / static_cast<int>(num_elements)) * static_cast<int>(i + 1);
    msgs.push_back(createValidMonitoringFrameMsg(scan_counter, start_angle, end_angle));
  }
  return msgs;
}

static std::vector<data_conversion_layer::monitoring_frame::MessageStamped>
stampMonitoringFrameMsgs(const std::vector<data_conversion_layer::monitoring_frame::Message>& msgs, int64_t stamp)
{
  std::vector<data_conversion_layer::monitoring_frame::MessageStamped> stamped_msgs;
  std::for_each(msgs.begin(), msgs.end(), [&stamped_msgs, stamp](const auto& msg) {
    stamped_msgs.push_back(data_conversion_layer::monitoring_frame::MessageStamped(msg, stamp));
  });
  return stamped_msgs;
}

LaserScan createReferenceScan(const std::vector<data_conversion_layer::monitoring_frame::Message>& msgs,
                              int64_t reference_timestamp)
{
  return data_conversion_layer::LaserScanConverter::toLaserScan(stampMonitoringFrameMsgs(msgs, reference_timestamp));
}

data_conversion_layer::RawData createRawData(std::string dataString)
{
  data_conversion_layer::RawData write_buf;
  std::copy(dataString.begin(), dataString.end(), std::back_inserter(write_buf));
  return write_buf;
}

ACTION_P(OpenBarrier, barrier)
{
  barrier->release();
}

using namespace ::testing;

MATCHER_P(PointwiseDoubleEq, vec, "")
{
  return std::equal(vec.begin(), vec.end(), arg.begin(), arg.end(), [](const double& a, const double& b) {
    return Matches(DoubleEq(b))(a);
  });
}

MATCHER_P(ScanDataEqual, scan, "")
{
  return arg.getScanCounter() == scan.getScanCounter() && arg.getScanResolution() == scan.getScanResolution() &&
         arg.getMinScanAngle() == scan.getMinScanAngle() && arg.getMaxScanAngle() == scan.getMaxScanAngle() &&
         Matches(PointwiseDoubleEq(scan.getMeasurements()))(arg.getMeasurements()) &&
         Matches(PointwiseDoubleEq(scan.getIntensities()))(arg.getIntensities());
}

using namespace ::testing;

MATCHER_P2(TimestampInExpectedTimeframe, reference_scan, reference_timestamp, "")
{
  const int64_t elapsed_time{ util::getCurrentTime() - reference_timestamp };
  *result_listener << "where the elapsed time is " << elapsed_time << " nsec";
  return arg.getTimestamp() > reference_scan.getTimestamp() &&
         arg.getTimestamp() < (reference_scan.getTimestamp() + elapsed_time);
}

}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_INTEGRATIONTEST_HELPER_H
