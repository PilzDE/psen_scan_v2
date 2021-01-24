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
#ifndef PSEN_SCAN_V2_STANDALONE_SCANNER_CONFIGURATION_H
#define PSEN_SCAN_V2_STANDALONE_SCANNER_CONFIGURATION_H

#include <boost/optional.hpp>

#include "psen_scan_v2_standalone/default_parameters.h"
#include "psen_scan_v2_standalone/configuration/scan_range.h"

namespace psen_scan_v2_standalone
{
namespace configuration
{
/**
 * @brief Higher level data type storing the configuration details of the scanner like scanner IP, port,
 * scan range, etc.
 */
class ScannerConfiguration
{
private:
  ScannerConfiguration() = default;

public:
  uint32_t hostIp() const;
  uint16_t hostUDPPortData() const;
  uint16_t hostUDPPortControl() const;

  uint32_t clientIp() const;
  uint16_t scannerDataPort() const;
  uint16_t scannerControlPort() const;

  const configuration::DefaultScanRange& scanRange() const;

  bool diagnosticsEnabled() const;

private:
  friend class ScannerConfigurationBuilder;

private:
  bool isValid() const;

private:
  boost::optional<uint32_t> host_ip_;
  uint16_t host_data_port_{ constants::DATA_PORT_OF_HOST_DEVICE };
  uint16_t host_control_port_{ constants::CONTROL_PORT_OF_HOST_DEVICE };

  boost::optional<uint32_t> scanner_ip_;
  uint16_t scanner_data_port_{ constants::DATA_PORT_OF_SCANNER_DEVICE };
  uint16_t scanner_control_port_{ constants::CONTROL_PORT_OF_SCANNER_DEVICE };

  boost::optional<DefaultScanRange> scan_range_{};
  bool diagnostics_enabled_{ false };
};

}  // namespace configuration
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_CONFIGURATION_H
