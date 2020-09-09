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

#ifndef PSEN_SCAN_V2_LASERSCAN_H
#define PSEN_SCAN_V2_LASERSCAN_H

#include <vector>
#include <cstdint>

namespace psen_scan_v2
{
using MeasurementData = std::vector<uint16_t>;

/**
 * @brief Class to hold the data for one laserscan without depencies to ROS.
 *
 */
class LaserScan
{
public:
  /**
   * @brief Construct a new Laser Scan object
   *
   * @param resolution Distance of angle between the measurements in radian.
   * @param min_scan_angle Lowest  Angle the Scanner is scanning in radian.
   * @param max_scan_angle Highest Angle the Scanner is scanning in radian.
   */
  LaserScan(const double& resolution, const double& min_scan_angle, const double& max_scan_angle);

public:
  const double& getScanResolution() const;
  const double& getMinScanAngle() const;
  const double& getMaxScanAngle() const;

  const MeasurementData& getMeasurements() const;
  MeasurementData& getMeasurements();

  bool isNumberOfScansValid() const;

private:
  //! Measurement data of the laserscan in Millimeters.
  MeasurementData measures_;
  //! Distance of angle between the measurements in radian.
  double resolution_;
  //! Lowest  Angle the Scanner is scanning in radian.
  const double min_scan_angle_;
  //! Highest Angle the Scanner is scanning in radian.
  const double max_scan_angle_;
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_LASERSCAN_H
