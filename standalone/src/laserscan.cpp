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
#include <ostream>
#include <stdexcept>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/util/format_range.h"

namespace psen_scan_v2_standalone
{
static const util::TenthOfDegree MAX_X_AXIS_ROTATION{ 275 };

LaserScan::LaserScan(const util::TenthOfDegree& resolution,
                     const util::TenthOfDegree& min_scan_angle,
                     const util::TenthOfDegree& max_scan_angle,
                     const uint32_t scan_counter,
                     const uint8_t active_zoneset,
                     const int64_t timestamp)
  : resolution_(resolution)
  , min_scan_angle_(min_scan_angle)
  , max_scan_angle_(max_scan_angle)
  , scan_counter_(scan_counter)
  , active_zoneset_(active_zoneset)
  , timestamp_(timestamp)
{
  if (scanResolution() == util::TenthOfDegree(0))
  {
    throw std::invalid_argument("Resolution must not be 0");
  }

  if (scanResolution() > MAX_X_AXIS_ROTATION)
  {
    throw std::invalid_argument("Resolution out of possible angle range");
  }

  if (minScanAngle() > maxScanAngle())
  {
    throw std::invalid_argument("Attention: Start angle has to be smaller or equal to the end angle!");
  }
}

const util::TenthOfDegree& LaserScan::scanResolution() const
{
  return resolution_;
}

// LCOV_EXCL_START
const util::TenthOfDegree& LaserScan::getScanResolution() const
{
  return this->scanResolution();
}

const util::TenthOfDegree& LaserScan::getMinScanAngle() const
{
  return this->minScanAngle();
}

const util::TenthOfDegree& LaserScan::getMaxScanAngle() const
{
  return this->maxScanAngle();
}

const LaserScan::MeasurementData& LaserScan::getMeasurements() const
{
  return this->measurements();
}

uint32_t LaserScan::getScanCounter() const
{
  return this->scanCounter();
}

int64_t LaserScan::getTimestamp() const
{
  return this->timestamp();
}

uint8_t LaserScan::getActiveZoneset() const
{
  return this->activeZoneset();
}

void LaserScan::setMeasurements(const MeasurementData& measurements)
{
  this->measurements(measurements);
}

LaserScan::MeasurementData& LaserScan::getMeasurements()
{
  return this->measurements();
}

const LaserScan::IntensityData& LaserScan::getIntensities() const
{
  return this->intensities();
}

void LaserScan::setIntensities(const IntensityData& intensities)
{
  this->intensities(intensities);
}
// LCOV_EXCL_STOP

const util::TenthOfDegree& LaserScan::minScanAngle() const
{
  return min_scan_angle_;
}

const util::TenthOfDegree& LaserScan::maxScanAngle() const
{
  return max_scan_angle_;
}

const LaserScan::MeasurementData& LaserScan::measurements() const
{
  return measurements_;
}

uint32_t LaserScan::scanCounter() const
{
  return scan_counter_;
}

uint8_t LaserScan::activeZoneset() const
{
  return active_zoneset_;
}

int64_t LaserScan::timestamp() const
{
  return timestamp_;
}

void LaserScan::measurements(const MeasurementData& measurements)
{
  measurements_ = measurements;
}

LaserScan::MeasurementData& LaserScan::measurements()
{
  return measurements_;
}

const LaserScan::IntensityData& LaserScan::intensities() const
{
  return intensities_;
}

void LaserScan::intensities(const IntensityData& intensities)
{
  intensities_ = intensities;
}

// LCOV_EXCL_START
void LaserScan::setIOStates(const IOData& io_states)
{
  ioStates(io_states);
}

const LaserScan::IOData& LaserScan::getIOStates() const
{
  return ioStates();
}
// LCOV_EXCL_STOP

void LaserScan::ioStates(const IOData& io_states)
{
  io_states_ = io_states;
}

const LaserScan::IOData& LaserScan::ioStates() const
{
  return io_states_;
}

std::ostream& operator<<(std::ostream& os, const LaserScan& scan)
{
  os << fmt::format("LaserScan(timestamp = {} nsec, scanCounter = {}, minScanAngle = {} deg, maxScanAngle = {} deg, "
                    "resolution = {} deg, active_zoneset = {}, measurements = {}, intensities = {}, io_states = {})",
                    scan.timestamp(),
                    scan.scanCounter(),
                    scan.minScanAngle().value() / 10.,
                    scan.maxScanAngle().value() / 10.,
                    scan.scanResolution().value() / 10.,
                    scan.activeZoneset(),
                    util::formatRange(scan.measurements()),
                    util::formatRange(scan.intensities()),
                    util::formatRange(scan.ioStates()));
  return os;
}

}  // namespace psen_scan_v2_standalone
