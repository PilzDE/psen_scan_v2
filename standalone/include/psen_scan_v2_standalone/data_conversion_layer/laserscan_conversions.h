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

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"

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
  static LaserScan toLaserScan(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames);

private:
  static std::vector<int>
  getFilledFramesIndicesSortedByThetaAngle(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames);
  static util::TenthOfDegree
  calculateMaxAngle(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames,
                    const util::TenthOfDegree& min_angle);
  static void validateMonitoringFrames(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames,
                                       const std::vector<int>& sorted_frames_indices);
  static bool allResolutionsMatch(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames);
  static bool allScanCountersMatch(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames);
  static bool thetaAnglesFitTogether(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames,
                                     const std::vector<int>& sorted_frames_indices);
};

inline LaserScan
LaserScanConverter::toLaserScan(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames)
{
  if (frames.empty())
  {
    throw ScannerProtocolViolationError("At least one monitoring frame is necessary to create a LaserScan");
  }

  std::vector<int> sorted_frames_indices = getFilledFramesIndicesSortedByThetaAngle(frames);
  validateMonitoringFrames(frames, sorted_frames_indices);

  const auto min_angle = frames[sorted_frames_indices[0]].fromTheta();
  const auto max_angle = calculateMaxAngle(frames, min_angle);

  std::vector<double> measurements;
  std::vector<double> intensities;

  for (auto index : sorted_frames_indices)
  {
    measurements.insert(measurements.end(), frames[index].measurements().begin(), frames[index].measurements().end());
    intensities.insert(intensities.end(), frames[index].intensities().begin(), frames[index].intensities().end());
  }

  LaserScan scan(frames[0].resolution(), min_angle, max_angle);
  scan.setMeasurements(measurements);
  scan.setIntensities(intensities);

  return scan;
}

inline std::vector<int> LaserScanConverter::getFilledFramesIndicesSortedByThetaAngle(
    const std::vector<data_conversion_layer::monitoring_frame::Message>& frames)
{
  std::vector<int> sorted_filled_frames_indices(frames.size());
  std::iota(sorted_filled_frames_indices.begin(), sorted_filled_frames_indices.end(), 0);
  std::sort(sorted_filled_frames_indices.begin(), sorted_filled_frames_indices.end(), [frames](int i1, int i2) {
    return frames[i1].fromTheta() < frames[i2].fromTheta();
  });

  // The following contains a missing line in the coverage report, which does not make sense.
  // LCOV_EXCL_START
  sorted_filled_frames_indices.erase(std::remove_if(sorted_filled_frames_indices.begin(),
                                                    sorted_filled_frames_indices.end(),
                                                    [frames](int i) { return frames[i].measurements().empty(); }),
                                     sorted_filled_frames_indices.end());
  // LCOV_EXCL_STOP

  return sorted_filled_frames_indices;
}

inline util::TenthOfDegree
LaserScanConverter::calculateMaxAngle(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames,
                                      const util::TenthOfDegree& min_angle)
{
  const auto resolution = frames[0].resolution();
  const uint16_t number_of_samples =
      std::accumulate(frames.begin(), frames.end(), uint16_t{ 0 }, [](uint16_t total, const auto& frame) {
        return total + frame.measurements().size();
      });
  return min_angle + resolution * static_cast<int>(number_of_samples - 1);
}

inline void LaserScanConverter::validateMonitoringFrames(
    const std::vector<data_conversion_layer::monitoring_frame::Message>& frames,
    const std::vector<int>& sorted_frames_indices)
{
  if (!allResolutionsMatch(frames))
  {
    throw ScannerProtocolViolationError("The resolution of all monitoring frames has to be the same.");
  }
  else if (!allScanCountersMatch(frames))
  {
    throw ScannerProtocolViolationError("The scan counters of all monitoring frames have to be the same.");
  }
  else if (!thetaAnglesFitTogether(frames, sorted_frames_indices))
  {
    throw ScannerProtocolViolationError("The monitoring frame ranges do not cover the whole scan range");
  }
}

inline bool
LaserScanConverter::allResolutionsMatch(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames)
{
  const auto resolution = frames[0].resolution();
  return std::all_of(
      frames.begin(), frames.end(), [resolution](const auto& frame) { return frame.resolution() == resolution; });
}

inline bool
LaserScanConverter::allScanCountersMatch(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames)
{
  const auto scan_counter = frames[0].scanCounter();
  return std::all_of(
      frames.begin(), frames.end(), [scan_counter](const auto& frame) { return frame.scanCounter() == scan_counter; });
}

inline bool
LaserScanConverter::thetaAnglesFitTogether(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames,
                                           const std::vector<int>& sorted_filled_frames_indices)
{
  util::TenthOfDegree lastEnd = frames[sorted_filled_frames_indices[0]].fromTheta();
  for (auto index : sorted_filled_frames_indices)
  {
    const auto& frame = frames[index];
    if (lastEnd != frame.fromTheta())
    {
      return false;
    }
    lastEnd = frame.fromTheta() + frame.resolution() * static_cast<int>(frame.measurements().size());
  }
  return true;
}

}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_LASERSCAN_CONVERSIONS_H
