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
#ifndef PSEN_SCAN_V2_SCANNER_CONFIGURATION_H
#define PSEN_SCAN_V2_SCANNER_CONFIGURATION_H

#include <string>

#include <arpa/inet.h>

#include "psen_scan_v2/scan_range.h"

namespace psen_scan_v2
{
/**
 * @brief Higher level data type storing the configuration details of the scanner like scanner IP, port,
 * scan range, etc.
 */
class ScannerConfiguration
{
public:
  /**
   * @brief Construtor.
   *
   * @param host_ip IP address of the host.
   * @param host_udp_port_data Port on which monitoring frames (scans) should be received.
   * @param host_udp_port_control Port used to send commands (start/stop) and receive the corresponding replies.
   * @param device_ip IP address of the scanner.
   * @param scan_range Range in which measurements are taken.
   * @param diagnostics_enabled Request diagnostic data from the scanner?
   */
  ScannerConfiguration(const std::string& host_ip,
                       const int& host_udp_port_data,
                       const int& host_udp_port_control,
                       const std::string& device_ip,
                       const DefaultScanRange& scan_range,
                       const bool diagnostics_enabled);

public:
  uint32_t hostIp() const;

  uint16_t hostUDPPortData() const;
  uint16_t hostUDPPortControl() const;

  uint32_t clientIp() const;

  const DefaultScanRange& scanRange() const;

  bool diagnostics_enabled() const;

private:
  uint32_t host_ip_;
  uint16_t host_udp_port_data_;
  uint16_t host_udp_port_control_;

  uint32_t client_ip_;

  const DefaultScanRange scan_range_;
  bool diagnostics_enabled_{ false };
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_CONFIGURATION_H
