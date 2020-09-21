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

#include <stdexcept>
#include <cassert>
#include <limits>
#include <cmath>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/laserscan.h"

namespace psen_scan_v2
{
static constexpr double MAX_X_AXIS_ROTATION(degreeToRadian(360.));
static constexpr double MIN_X_AXIS_ROTATION(degreeToRadian(-360.));

LaserScan::LaserScan(const double& resolution, const double& min_scan_angle, const double& max_scan_angle)
  : resolution_(resolution), min_scan_angle_(min_scan_angle), max_scan_angle_(max_scan_angle)
{
  if (getScanResolution() == 0.)
  {
    throw std::invalid_argument("Resolution must not be 0");
  }

  if (getScanResolution() < MIN_X_AXIS_ROTATION || getScanResolution() > MAX_X_AXIS_ROTATION)
  {
    throw std::invalid_argument("Resolution out of possible angle range");
  }

  if (getMinScanAngle() >= getMaxScanAngle())
  {
    throw std::invalid_argument("Attention: Start angle has to be smaller than end angle!");
  }
}

const double& LaserScan::getScanResolution() const
{
  return resolution_;
}

const double& LaserScan::getMinScanAngle() const
{
  return min_scan_angle_;
}

const double& LaserScan::getMaxScanAngle() const
{
  return max_scan_angle_;
}

bool LaserScan::isValid() const
{
  assert(getMinScanAngle() < getMaxScanAngle() && "Invalid scan range");

  using size_type = MeasurementData::size_type;
  const auto angle_range{ getMaxScanAngle() - getMinScanAngle() };
  const size_type expected_size{ static_cast<size_type>(std::floor(angle_range / getScanResolution())) };
  return measures_.size() == expected_size;
}

const MeasurementData& LaserScan::getMeasurements() const
{
  return measures_;
}

void LaserScan::setMeasurements(const MeasurementData& measures)
{
  measures_ = measures;
}

MeasurementData& LaserScan::getMeasurements()
{
  return measures_;
}

bool LaserScan::operator==(const LaserScan& scan) const
{
  if ((this->getMaxScanAngle() != scan.getMaxScanAngle()) || (this->getMinScanAngle() != scan.getMinScanAngle()) ||
      (this->getScanResolution() != scan.getScanResolution()) ||
      (this->getMeasurements().size() != scan.getMeasurements().size()))
  {
    for (size_t i = 0; i < this->getMeasurements().size(); i++)
    {
      if (this->getMeasurements().at(i) != scan.getMeasurements().at(i))
      {
        return false;
      }
    }
  }

  return true;
}

}  // namespace psen_scan_v2
