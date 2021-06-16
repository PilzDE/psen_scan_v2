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
#include <cmath>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "psen_scan_v2_standalone/configuration/scanner_ids.h"
#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"
#include "psen_scan_v2_standalone/scan_range.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;

static constexpr ScanRange DEFAULT_SCAN_RANGE{ util::TenthOfDegree(1), util::TenthOfDegree(60) };
static constexpr util::TenthOfDegree DEFAULT_SCAN_RESOLUTION{ 2 };

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

  const unsigned int num_elements = (end_angle - start_angle) / resolution;
  const double lowest_measurement{ 0. };
  const double highest_measurement{ 10. };
  const std::vector<double> measurements{ generateMeasurements(num_elements, lowest_measurement, highest_measurement) };

  const double lowest_intensity{ 0. };
  const double highest_intensity{ 17000. };
  const std::vector<double> intensities{ generateIntensities(num_elements, lowest_intensity, highest_intensity) };

  const std::vector<data_conversion_layer::monitoring_frame::diagnostic::Message> diagnostic_messages{
    { configuration::ScannerId::master, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(1, 7) }
  };

  return data_conversion_layer::monitoring_frame::Message(
      start_angle, resolution, scan_counter, measurements, intensities, diagnostic_messages);
}

static std::vector<data_conversion_layer::monitoring_frame::Message>
createValidMonitoringFrameMsgs(const uint32_t scan_counter, const std::size_t num_elements)
{
  std::vector<data_conversion_layer::monitoring_frame::Message> msgs(num_elements);
  std::generate(msgs.begin(), msgs.end(), [scan_counter]() { return createValidMonitoringFrameMsg(scan_counter); });
  return msgs;
}

static std::vector<data_conversion_layer::monitoring_frame::Message>
createMonitoringFrameMsgsForScanRound(const uint32_t scan_counter, const std::size_t num_elements)
{
  std::vector<data_conversion_layer::monitoring_frame::Message> msgs(num_elements);
  for (std::size_t i = 0; i < msgs.size(); ++i)
  {
    const util::TenthOfDegree start_angle =
        (DEFAULT_SCAN_RANGE.getEnd() / static_cast<int>(num_elements)) * static_cast<int>(i);
    const util::TenthOfDegree end_angle =
        (DEFAULT_SCAN_RANGE.getEnd() / static_cast<int>(num_elements)) * static_cast<int>(i + 1);
    msgs[i] = createValidMonitoringFrameMsg(scan_counter, start_angle, end_angle);
  }
  return msgs;
}

ACTION_P(OpenBarrier, barrier)
{
  barrier->release();
}

}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_INTEGRATIONTEST_HELPER_H
