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
#ifndef PSEN_SCAN_V2_START_REQUEST_H
#define PSEN_SCAN_V2_START_REQUEST_H

#include <array>
#include <cstdint>

#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/scan_range.h"
#include "psen_scan_v2/scanner_ids.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/tenth_of_degree.h"

namespace psen_scan_v2
{
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

  friend DynamicSizeRawData start_request::serialize(const start_request::Message&, const uint32_t&);
  friend DynamicSizeRawData start_request::serialize(const start_request::Message&);

private:
  class LaserScanSettings
  {
  public:
    constexpr LaserScanSettings() = default;
    constexpr LaserScanSettings(const DefaultScanRange& scan_range, const TenthOfDegree& resolution);

  public:
    constexpr const DefaultScanRange& getScanRange() const;
    constexpr TenthOfDegree getResolution() const;

  private:
    const DefaultScanRange scan_range_{};
    const TenthOfDegree resolution_{ 0 };
  };

  class DeviceSettings
  {
  public:
    constexpr DeviceSettings(const ScannerId id, const bool diagnostics_enabled);

  public:
    constexpr bool isDiagnosticsEnabled() const;

  private:
    const ScannerId id_;
    const bool diagnostics_enabled_;
  };

private:
  static constexpr std::size_t NUM_SLAVES{ 3 };

private:
  const uint32_t host_ip_;
  const uint16_t host_udp_port_data_;

  const DeviceSettings master_device_settings_;
  const LaserScanSettings master_;
  const std::array<LaserScanSettings, NUM_SLAVES> slaves_;
};

constexpr Message::LaserScanSettings::LaserScanSettings(const DefaultScanRange& scan_range,
                                                        const TenthOfDegree& resolution)
  : scan_range_(scan_range), resolution_(resolution)
{
}

constexpr const DefaultScanRange& Message::LaserScanSettings::getScanRange() const
{
  return scan_range_;
};

constexpr TenthOfDegree Message::LaserScanSettings::getResolution() const
{
  return resolution_;
};

constexpr Message::DeviceSettings::DeviceSettings(const ScannerId id, const bool diagnostics_enabled)
  : id_(id), diagnostics_enabled_(diagnostics_enabled)
{
}

constexpr bool Message::DeviceSettings::isDiagnosticsEnabled() const
{
  return diagnostics_enabled_;
};

}  // namespace start_request
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_START_REQUEST_H
