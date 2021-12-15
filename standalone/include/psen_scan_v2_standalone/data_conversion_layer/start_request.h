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
#ifndef PSEN_SCAN_V2_STANDALONE_START_REQUEST_H
#define PSEN_SCAN_V2_STANDALONE_START_REQUEST_H

#include <array>
#include <cstdint>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/scan_range.h"
#include "psen_scan_v2_standalone/util/tenth_of_degree.h"

namespace psen_scan_v2_standalone
{
class ScannerConfiguration;

namespace data_conversion_layer
{
/**
 * @brief Contains all things needed to define and implement a data_conversion_layer::start_request::Message.
 */
namespace start_request
{
/**
 * @brief Higher level data type representing a scanner start request.
 *
 * @note Unless otherwise indicated the byte order is little endian.
 *
 */
class Message
{
public:
  Message(const ScannerConfiguration& scanner_configuration);

  friend data_conversion_layer::RawData serialize(const data_conversion_layer::start_request::Message& msg,
                                                  const uint32_t& seq_number);

private:
  /**
   * @brief Class describing the scan settings of the master and slave devices.
   *
   * The settings include the scan range and the scan resolution of the respective device.
   */
  class LaserScanSettings
  {
  public:
    constexpr LaserScanSettings() = default;
    constexpr LaserScanSettings(const ScanRange& scan_range, const util::TenthOfDegree& resolution);

  public:
    /*! deprecated: use constexpr const ScanRange& scanRange() const instead */
    [[deprecated("use constexpr const ScanRange& scanRange() const instead")]] constexpr const ScanRange&
    getScanRange() const;

    constexpr const ScanRange& scanRange() const;

    /*! deprecated: use constexpr util::TenthOfDegree resolution() const instead */
    [[deprecated("use constexpr util::TenthOfDegree resolution() const instead")]] constexpr util::TenthOfDegree
    getResolution() const;

    constexpr util::TenthOfDegree resolution() const;

  private:
    const ScanRange scan_range_{ ScanRange::createInvalidScanRange() };
    const util::TenthOfDegree resolution_{ 0 };
  };

  /**
   * @brief Class describing the fundamental settings of the master and slave devices, like id and diagnostics.
   */
  class DeviceSettings
  {
  public:
    constexpr DeviceSettings(const bool diagnostics_enabled, const bool intensities_enabled);

  public:
    constexpr bool diagnosticsEnabled() const;
    constexpr bool intensitiesEnabled() const;

  private:
    const bool diagnostics_enabled_;
    const bool intensities_enabled_;
  };

private:
  static constexpr std::size_t NUM_SLAVES{ 3 };

private:
  const uint32_t host_ip_;  ///< network byte order = big endian
  const uint16_t host_udp_port_data_;

  const DeviceSettings master_device_settings_;
  const LaserScanSettings master_;
  const std::array<LaserScanSettings, NUM_SLAVES> slaves_;
};

constexpr Message::LaserScanSettings::LaserScanSettings(const ScanRange& scan_range,
                                                        const util::TenthOfDegree& resolution)
  : scan_range_(scan_range), resolution_(resolution)
{
}

constexpr const ScanRange& Message::LaserScanSettings::scanRange() const
{
  return scan_range_;
};

constexpr util::TenthOfDegree Message::LaserScanSettings::resolution() const
{
  return resolution_;
};

constexpr const ScanRange& Message::LaserScanSettings::getScanRange() const
{
  return this->scanRange();
};

constexpr util::TenthOfDegree Message::LaserScanSettings::getResolution() const
{
  return this->resolution();
};

constexpr Message::DeviceSettings::DeviceSettings(const bool diagnostics_enabled, const bool intensities_enabled)
  : diagnostics_enabled_(diagnostics_enabled), intensities_enabled_(intensities_enabled)
{
}

constexpr bool Message::DeviceSettings::diagnosticsEnabled() const
{
  return diagnostics_enabled_;
};

constexpr bool Message::DeviceSettings::intensitiesEnabled() const
{
  return intensities_enabled_;
};

}  // namespace start_request
}  // namespace data_conversion_layer
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_START_REQUEST_H
