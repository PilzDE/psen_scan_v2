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

#include <algorithm>
#include <stdexcept>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/laserscan.h"

namespace psen_scan_v2_standalone
{
static const util::TenthOfDegree MAX_X_AXIS_ROTATION{ 275 };

LaserScan::LaserScan(const util::TenthOfDegree& resolution,
                     const util::TenthOfDegree& min_scan_angle,
                     const util::TenthOfDegree& max_scan_angle)
  : resolution_(resolution), min_scan_angle_(min_scan_angle), max_scan_angle_(max_scan_angle)
{
  if (getScanResolution() == util::TenthOfDegree(0))
  {
    throw std::invalid_argument("Resolution must not be 0");
  }

  if (getScanResolution() > MAX_X_AXIS_ROTATION)
  {
    throw std::invalid_argument("Resolution out of possible angle range");
  }

  if (getMinScanAngle() > getMaxScanAngle())
  {
    throw std::invalid_argument("Attention: Start angle has to be smaller or equal to the end angle!");
  }
}

const util::TenthOfDegree& LaserScan::getScanResolution() const
{
  return resolution_;
}

const util::TenthOfDegree& LaserScan::getMinScanAngle() const
{
  return min_scan_angle_;
}

const util::TenthOfDegree& LaserScan::getMaxScanAngle() const
{
  return max_scan_angle_;
}

const LaserScan::MeasurementData& LaserScan::getMeasurements() const
{
  return measurements_;
}

void LaserScan::setMeasurements(const MeasurementData& measurements)
{
  measurements_ = measurements;
}

LaserScan::MeasurementData& LaserScan::getMeasurements()
{
  return measurements_;
}

const LaserScan::IntensityData& LaserScan::getIntensities() const
{
  return intensities_;
}

void LaserScan::setIntensities(const IntensityData& intensities)
{
  intensities_ = intensities;
}

bool LaserScan::operator==(const LaserScan& scan) const
{
  return ((max_scan_angle_ == scan.max_scan_angle_) && (min_scan_angle_ == scan.min_scan_angle_) &&
          (resolution_ == scan.resolution_) && (measurements_.size() == scan.measurements_.size()) &&
          std::equal(measurements_.begin(), measurements_.end(), scan.measurements_.begin()));
}

}  // namespace psen_scan_v2_standalone
