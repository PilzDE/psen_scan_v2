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

#include "psen_scan_v2_standalone/scanner_configuration.h"

namespace psen_scan_v2_standalone
{
bool ScannerConfiguration::isValid() const
{
  // clang-format off
  return host_ip_     && host_data_port_    && host_control_port_ &&
         scanner_ip_  && scanner_data_port_ && scanner_control_port_ &&
         scan_range_;
  // clang-format on
}

uint32_t ScannerConfiguration::hostIp() const
{
  return *host_ip_;
}

uint16_t ScannerConfiguration::hostUDPPortData() const
{
  return *host_data_port_;
}

uint16_t ScannerConfiguration::hostUDPPortControl() const
{
  return *host_control_port_;
}

uint32_t ScannerConfiguration::clientIp() const
{
  return *scanner_ip_;
}

uint16_t ScannerConfiguration::scannerDataPort() const
{
  return *scanner_data_port_;
}

uint16_t ScannerConfiguration::scannerControlPort() const
{
  return *scanner_control_port_;
}

const DefaultScanRange& ScannerConfiguration::scanRange() const
{
  return *scan_range_;
}

bool ScannerConfiguration::diagnosticsEnabled() const
{
  return diagnostics_enabled_;
}

}  // namespace psen_scan_v2_standalone
