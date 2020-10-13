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
#include <string>

#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/scan_range.h"

namespace psen_scan_v2
{
static constexpr std::size_t START_REQUEST_SIZE{ 58 };  // See protocol description

/**
 * @brief Higher level data type representing a scanner start request.
 *
 * @note Unless otherwise indicated the byte order is little endian.
 *
 */
class StartRequest
{
public:
  /**
   * @brief Constructor.
   *
   * @param scanner_configuration Specifies the required scanner configuration.
   * @param seq_number TODO
   */
  StartRequest(const ScannerConfiguration& scanner_configuration, const uint32_t& seq_number);

  //! @returns the CRC of the start request.
  uint32_t getCRC() const;

  DynamicSizeRawData serialize() const;

private:
  uint32_t crc_{ 0 }; /**< Will be filled in constructor */
  uint32_t seq_number_;
  uint64_t const RESERVED_{ 0 };           /**< Use all zeros */
  uint32_t const OPCODE_{ htole32(0x35) }; /**< Constant 0x35. */
  uint32_t host_ip_;                       /**< Byte order: big endian */
  uint16_t host_udp_port_data_;            /**< Byte order: big endian */

  /**< The following 'enable' fields are a 1-byte mask each.
   * Only the last 4 bits (little endian) are used, each of which represents a device.
   * For example, (1000) only enables the Master device, while (1010) enables both the Master
   * and the second Slave device.
   */
  uint8_t device_enabled_{ 0b00001000 };
  uint8_t intensity_enabled_{ 0 };
  uint8_t point_in_safety_enabled_{ 0 };
  uint8_t active_zone_set_enabled_{ 0 };
  uint8_t io_pin_enabled_{ 0 };
  uint8_t scan_counter_enabled_{ 0b00001000 };
  uint8_t speed_encoder_enabled_{ 0 }; /**< 0000000bin disabled, 00001111bin enabled.*/
  uint8_t diagnostics_enabled_{ 0 };

  class DeviceField
  {
  public:
    DeviceField() = default;

    DeviceField(const DefaultScanRange& scan_range, const TenthOfDegree resolution)
      : scan_range_(scan_range), resolution_(resolution)
    {
    }

  public:
    const DefaultScanRange& getScanRange() const
    {
      return scan_range_;
    };

    TenthOfDegree getResolution() const
    {
      return resolution_;
    };

  private:
    const DefaultScanRange scan_range_{};
    TenthOfDegree resolution_{ 0 };
  };

  DeviceField master_;
  std::array<DeviceField, 3> slaves_;
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_START_REQUEST_H
