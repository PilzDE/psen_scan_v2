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

namespace psen_scan_v2
{
class ScannerConfiguration
{
public:
  ScannerConfiguration(const std::string& host_ip,
                       const int& host_udp_port_data,
                       const int& host_udp_port_control,
                       const std::string& device_ip,
                       const double& start_angle,
                       const double& end_angle);

public:
  uint32_t hostIp() const;

  uint16_t hostUDPPortData() const;
  uint16_t hostUDPPortControl() const;

  uint32_t clientIp() const;

  double startAngle() const;
  double endAngle() const;

private:
  uint32_t host_ip_;
  uint16_t host_udp_port_data_;
  uint16_t host_udp_port_control_;

  uint32_t client_ip_;

  //! Angle in rad.
  double start_angle_;
  //! Angle in rad.
  double end_angle_;
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_CONFIGURATION_H
