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

#include "psen_scan_v2/tenth_of_degree.h"

namespace psen_scan_v2
{
/**
 *  @brief This class represents a single laser scan in the \<prefix\>_scan target frame.
 *
 * All relevant information about a single scan of a psen_scan scanner are stored in this class.
 * This includes the following information:
 *
 * - Distance measures in mm.
 * - Normalized intensities of the signals.
 * - Resolution of the scan in radians.
 * - Min and Max angle in radians.
 *
 * The measures use the target frame defined as <prefix>_scan.
 * @see https://github.com/PilzDE/psen_scan_v2/blob/main/README.md#tf-frames
 */
class LaserScan
{
public:
  using MeasurementData = std::vector<double>;
  using IntensityData = std::vector<double>;

public:
  LaserScan(const TenthOfDegree& resolution, const TenthOfDegree& min_scan_angle, const TenthOfDegree& max_scan_angle);

public:
  const TenthOfDegree& getScanResolution() const;
  const TenthOfDegree& getMinScanAngle() const;
  const TenthOfDegree& getMaxScanAngle() const;

  const MeasurementData& getMeasurements() const;
  MeasurementData& getMeasurements();
  void setMeasurements(const MeasurementData&);

  const IntensityData& getIntensities() const;
  void setIntensities(const IntensityData&);

  bool operator==(const LaserScan& scan) const;

private:
  //! Measurement data of the laserscan (in Millimeters).
  MeasurementData measures_;
  //! Stores the received normalized signal intensities.
  IntensityData intensities_;
  //! Distance of angle between the measurements (in radian).
  const TenthOfDegree resolution_;
  //! Lowest angle the scanner is scanning (in radian).
  const TenthOfDegree min_scan_angle_;
  //! Highest angle the scanner is scanning (in radian).
  const TenthOfDegree max_scan_angle_;
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_LASERSCAN_H
