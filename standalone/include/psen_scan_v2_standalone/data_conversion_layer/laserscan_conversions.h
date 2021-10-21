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

#ifndef PSEN_SCAN_V2_STANDALONE_LASERSCAN_CONVERSIONS_H
#define PSEN_SCAN_V2_STANDALONE_LASERSCAN_CONVERSIONS_H

#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/laserscan.h"

#include <algorithm>
#include <numeric>
#include <vector>

namespace psen_scan_v2_standalone
{
namespace data_conversion_layer
{
/**
 * @brief: Exception thrown if data received from the scanner hardware could not be processed according to protocol.
 */
class ScannerProtocolViolationError : public std::runtime_error
{
public:
  ScannerProtocolViolationError(const std::string& msg) : std::runtime_error(msg){};
};

/**
 * @brief: Responsible for converting Monitoring frames into LaserScan messages.
 */
class LaserScanConverter
{
public:
  /**
   * @brief Converts monitoring_frames of a scan_round to the user friendly LaserScan type sent by the
   * IScanner::LaserScanCallback.
   *
   * @note expects all monitoring frames to have the same resolution.
   *
   * @see data_conversion_layer::monitoring_frame::Message
   * @see ScannerV2
   */
  static LaserScan
  toLaserScan(const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs);

private:
  static std::vector<int> getFilledFramesIndicesSortedByThetaAngle(
      const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs);
  static util::TenthOfDegree
  calculateMaxAngle(const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs,
                    const util::TenthOfDegree& min_angle);
  static int64_t
  calculateTimestamp(const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs,
                     const std::vector<int>& filled_stamped_msgs_indices);
  static int64_t calculateFirstRayTime(const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg);
  static void
  validateMonitoringFrames(const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs,
                           const std::vector<int>& sorted_stamped_msgs_indices);
  static bool
  allResolutionsMatch(const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs);
  static bool
  allScanCountersMatch(const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs);
  static bool
  thetaAnglesFitTogether(const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs,
                         const std::vector<int>& sorted_stamped_msgs_indices);
};

inline LaserScan LaserScanConverter::toLaserScan(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs)
{
  if (stamped_msgs.empty())
  {
    throw ScannerProtocolViolationError("At least one monitoring frame is necessary to create a LaserScan");
  }

  std::vector<int> sorted_stamped_msgs_indices = getFilledFramesIndicesSortedByThetaAngle(stamped_msgs);
  validateMonitoringFrames(stamped_msgs, sorted_stamped_msgs_indices);

  const auto min_angle = stamped_msgs[sorted_stamped_msgs_indices[0]].msg_.fromTheta();
  const auto max_angle = calculateMaxAngle(stamped_msgs, min_angle);

  const auto timestamp = calculateTimestamp(stamped_msgs, sorted_stamped_msgs_indices);

  std::vector<double> measurements;
  std::vector<double> intensities;

  for (auto index : sorted_stamped_msgs_indices)
  {
    measurements.insert(measurements.end(),
                        stamped_msgs[index].msg_.measurements().begin(),
                        stamped_msgs[index].msg_.measurements().end());
    intensities.insert(intensities.end(),
                       stamped_msgs[index].msg_.intensities().begin(),
                       stamped_msgs[index].msg_.intensities().end());
  }

  LaserScan scan(stamped_msgs[0].msg_.resolution(),
                 min_angle,
                 max_angle,
                 stamped_msgs[0].msg_.scanCounter(),
                 stamped_msgs[sorted_stamped_msgs_indices.back()].msg_.activeZoneset(),
                 timestamp);
  scan.setMeasurements(measurements);
  scan.setIntensities(intensities);

  return scan;
}

inline std::vector<int> LaserScanConverter::getFilledFramesIndicesSortedByThetaAngle(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs)
{
  std::vector<int> sorted_filled_stamped_msgs_indices(stamped_msgs.size());
  std::iota(sorted_filled_stamped_msgs_indices.begin(), sorted_filled_stamped_msgs_indices.end(), 0);
  std::sort(sorted_filled_stamped_msgs_indices.begin(),
            sorted_filled_stamped_msgs_indices.end(),
            [&stamped_msgs](int i1, int i2) {
              return stamped_msgs[i1].msg_.fromTheta() < stamped_msgs[i2].msg_.fromTheta();
            });

  // The following contains a missing line in the coverage report, which does not make sense.
  // LCOV_EXCL_START
  sorted_filled_stamped_msgs_indices.erase(
      std::remove_if(sorted_filled_stamped_msgs_indices.begin(),
                     sorted_filled_stamped_msgs_indices.end(),
                     [&stamped_msgs](int i) { return stamped_msgs[i].msg_.measurements().empty(); }),
      sorted_filled_stamped_msgs_indices.end());
  // LCOV_EXCL_STOP

  return sorted_filled_stamped_msgs_indices;
}

inline util::TenthOfDegree LaserScanConverter::calculateMaxAngle(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs,
    const util::TenthOfDegree& min_angle)
{
  const auto resolution = stamped_msgs[0].msg_.resolution();
  const uint16_t number_of_samples = std::accumulate(
      stamped_msgs.begin(), stamped_msgs.end(), uint16_t{ 0 }, [](uint16_t total, const auto& stamped_msg) {
        return total + stamped_msg.msg_.measurements().size();
      });
  return min_angle + resolution * static_cast<int>(number_of_samples - 1);
}

inline int64_t LaserScanConverter::calculateTimestamp(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs,
    const std::vector<int>& filled_stamped_msgs_indices)
{
  const auto it = std::min_element(
      filled_stamped_msgs_indices.begin(), filled_stamped_msgs_indices.end(), [&stamped_msgs](int i, int j) {
        return stamped_msgs[i].stamp_ < stamped_msgs[j].stamp_;
      });  // determines stamped_msg with smallest stamp
  return calculateFirstRayTime(stamped_msgs[*it]);
}

inline int64_t
LaserScanConverter::calculateFirstRayTime(const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg)
{
  const double time_per_scan_in_ns{ configuration::TIME_PER_SCAN_IN_S * 1000000000.0 };
  const double scan_interval_in_degree{ stamped_msg.msg_.resolution().value() *
                                        (stamped_msg.msg_.measurements().size() - 1) / 10.0 };
  return stamped_msg.stamp_ - static_cast<int64_t>(std::round(scan_interval_in_degree * time_per_scan_in_ns / 360.0));
}

inline void LaserScanConverter::validateMonitoringFrames(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs,
    const std::vector<int>& sorted_stamped_msgs_indices)
{
  if (!allResolutionsMatch(stamped_msgs))
  {
    throw ScannerProtocolViolationError("The resolution of all monitoring frames has to be the same.");
  }
  else if (!allScanCountersMatch(stamped_msgs))
  {
    throw ScannerProtocolViolationError("The scan counters of all monitoring frames have to be the same.");
  }
  else if (!thetaAnglesFitTogether(stamped_msgs, sorted_stamped_msgs_indices))
  {
    throw ScannerProtocolViolationError("The monitoring frame ranges do not cover the whole scan range");
  }
}

inline bool LaserScanConverter::allResolutionsMatch(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs)
{
  const auto resolution = stamped_msgs[0].msg_.resolution();
  return std::all_of(stamped_msgs.begin(), stamped_msgs.end(), [resolution](const auto& stamped_msg) {
    return stamped_msg.msg_.resolution() == resolution;
  });
}

inline bool LaserScanConverter::allScanCountersMatch(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs)
{
  const auto scan_counter = stamped_msgs[0].msg_.scanCounter();
  return std::all_of(stamped_msgs.begin(), stamped_msgs.end(), [scan_counter](const auto& stamped_msg) {
    return stamped_msg.msg_.scanCounter() == scan_counter;
  });
}

inline bool LaserScanConverter::thetaAnglesFitTogether(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs,
    const std::vector<int>& sorted_filled_stamped_msgs_indices)
{
  util::TenthOfDegree lastEnd = stamped_msgs[sorted_filled_stamped_msgs_indices[0]].msg_.fromTheta();
  for (auto index : sorted_filled_stamped_msgs_indices)
  {
    const auto& stamped_msg = stamped_msgs[index];
    if (lastEnd != stamped_msg.msg_.fromTheta())
    {
      return false;
    }
    lastEnd = stamped_msg.msg_.fromTheta() +
              stamped_msg.msg_.resolution() * static_cast<int>(stamped_msg.msg_.measurements().size());
  }
  return true;
}

}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_LASERSCAN_CONVERSIONS_H
