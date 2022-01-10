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

#ifndef PSEN_SCAN_V2_STANDALONE_LASERSCAN_H
#define PSEN_SCAN_V2_STANDALONE_LASERSCAN_H

#include <cstdint>
#include <ostream>
#include <vector>

#include "psen_scan_v2_standalone/io_state.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"

namespace psen_scan_v2_standalone
{
/**
 *  @brief This class represents a single laser scan in the \<tf_prefix\> target frame.
 *
 * All relevant information about a single scan of a psen_scan scanner are stored in this class.
 * This includes the following information:
 *
 * - Distance measures in mm.
 * - Normalized intensities of the signals.
 * - Resolution of the scan.
 * - Min and Max angle.
 * - Counter of scan round.
 * - ID of the currently active zoneset.
 * - Time of the first scan ray.
 * - All states of the I/O pins recorded during the scan.
 *
 * The measures use the target frame defined as \<tf_prefix\>.
 * @see https://github.com/PilzDE/psen_scan_v2_standalone/blob/main/README.md#tf-frames
 */
class LaserScan
{
public:
  using MeasurementData = std::vector<double>;
  using IntensityData = std::vector<double>;
  using IOData = std::vector<IOState>;

public:
  LaserScan(const util::TenthOfDegree& resolution,
            const util::TenthOfDegree& min_scan_angle,
            const util::TenthOfDegree& max_scan_angle,
            const uint32_t scan_counter,
            const uint8_t active_zoneset,
            const int64_t timestamp);

public:
  /*! deprecated: use const util::TenthOfDegree& scanResolution() const instead */
  [[deprecated("use const util::TenthOfDegree& scanResolution() const instead")]] const util::TenthOfDegree&
  getScanResolution() const;
  const util::TenthOfDegree& scanResolution() const;

  /*! deprecated: use const util::TenthOfDegree& minScanAngle() const instead */
  [[deprecated("use const util::TenthOfDegree& minScanAngle() const instead")]] const util::TenthOfDegree&
  getMinScanAngle() const;
  const util::TenthOfDegree& minScanAngle() const;

  /*! deprecated: use const util::TenthOfDegree& maxScanAngle() const instead */
  [[deprecated("use const util::TenthOfDegree& maxScanAngle() const instead")]] const util::TenthOfDegree&
  getMaxScanAngle() const;
  const util::TenthOfDegree& maxScanAngle() const;

  /*! deprecated: use uint32_t scanCounter() const instead */
  [[deprecated("use uint32_t scanCounter() const instead")]] uint32_t getScanCounter() const;
  uint32_t scanCounter() const;

  /*! deprecated: use uint8_t activeZoneset() const instead */
  [[deprecated("use uint8_t activeZoneset() const instead")]] uint8_t getActiveZoneset() const;
  uint8_t activeZoneset() const;

  /*! deprecated: use int64_t timestamp() const instead */
  [[deprecated("use int64_t timestamp() const instead")]] int64_t getTimestamp() const;
  int64_t timestamp() const;

  /*! deprecated: use const MeasurementData& measurements() instead */
  [[deprecated("use const MeasurementData& measurements() const instead")]] const MeasurementData&
  getMeasurements() const;
  const MeasurementData& measurements() const;

  /*! deprecated: use MeasurementData& measurements() instead */
  [[deprecated("use MeasurementData& measurements() instead")]] MeasurementData& getMeasurements();
  MeasurementData& measurements();

  /*! deprecated: use void measurements(const MeasurementData& measurements) instead */
  [[deprecated("use void measurements(const MeasurementData& measurements) instead")]] void
  setMeasurements(const MeasurementData& measurements);
  void measurements(const MeasurementData& measurements);

  /*! deprecated: use const IntensityData& intensities() instead */
  [[deprecated("use const IntensityData& intensities() const instead")]] const IntensityData& getIntensities() const;
  const IntensityData& intensities() const;

  /*! deprecated: use void intensities(const IntensityData& intensities) instead */
  [[deprecated("use void intensities(const IntensityData& intensities)) instead")]] void
  setIntensities(const IntensityData& intensities);
  void intensities(const IntensityData& intensities);

  /*! deprecated: use const IOData& ioStates() const instead */
  [[deprecated("use const IOData& ioStates() const instead")]] const IOData& getIOStates() const;
  const IOData& ioStates() const;

  /*! deprecated: use void ioStates(const IOData& io_states) instead */
  [[deprecated("use void ioStates(const IOData& io_states) instead")]] void setIOStates(const IOData& io_states);
  void ioStates(const IOData& io_states);

private:
  //! Measurement data of the laserscan (in Millimeters).
  MeasurementData measurements_;
  //! Stores the received normalized signal intensities.
  IntensityData intensities_;
  //! States of the I/O pins.
  IOData io_states_;
  //! Distance of angle between the measurements.
  const util::TenthOfDegree resolution_;
  //! Lowest angle the scanner is scanning.
  const util::TenthOfDegree min_scan_angle_;
  //! Highest angle the scanner is scanning.
  const util::TenthOfDegree max_scan_angle_;
  //! Number of the scan round this data belongs to.
  const uint32_t scan_counter_;
  //! The currently active zoneset of the scanner.
  const uint8_t active_zoneset_;
  //! Time of the first ray in this scan round (or fragment if fragmented_scans is enabled).
  const int64_t timestamp_;
};

std::ostream& operator<<(std::ostream& os, const LaserScan& scan);

}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_LASERSCAN_H
