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
 * @brief Converts monitoring_frames of a scan_round to the user friendly LaserScan type sent by the
 * IScanner::LaserScanCallback.
 *
 * @note expects all monitoring frames to have the same resolution.
 *
 * @see data_conversion_layer::monitoring_frame::Message
 * @see ScannerV2
 */
static LaserScan toLaserScan(const std::vector<data_conversion_layer::monitoring_frame::Message>& frames)
{
  assert(!frames.empty());
  const auto resolution = frames[0].resolution();
  assert(std::all_of(
      frames.begin(), frames.end(), [resolution](const auto& frame) { return frame.resolution() == resolution; }));

  std::vector<int> sorted_frames_indices(frames.size());
  std::iota(sorted_frames_indices.begin(), sorted_frames_indices.end(), 0);
  std::sort(sorted_frames_indices.begin(), sorted_frames_indices.end(), [frames](int i1, int i2) {
    return frames[i1].fromTheta() < frames[i2].fromTheta();
  });

  const auto min_angle = frames[sorted_frames_indices[0]].fromTheta();
  const uint16_t number_of_samples =
      std::accumulate(frames.begin(), frames.end(), uint16_t{ 0 }, [](uint16_t total, const auto& frame) {
        return total + frame.measurements().size();
      });
  const auto max_angle = (min_angle + resolution * static_cast<int>(number_of_samples));

  std::vector<double> measurements;
  std::vector<double> intensities;

  for (auto index : sorted_frames_indices)
  {
    measurements.insert(measurements.end(), frames[index].measurements().begin(), frames[index].measurements().end());
    intensities.insert(intensities.end(), frames[index].intensities().begin(), frames[index].intensities().end());
  }

  LaserScan scan(resolution, min_angle, max_angle);
  scan.setMeasurements(measurements);
  scan.setIntensities(intensities);

  return scan;
}

}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_LASERSCAN_CONVERSIONS_H
