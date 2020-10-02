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

#include <cmath>
#include <cassert>
#include <stdexcept>
#include <limits>

#include <arpa/inet.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/scanner_configuration.h"

namespace psen_scan_v2
{
ScannerConfiguration::ScannerConfiguration(const std::string& host_ip,
                                           const int& host_udp_port_data,
                                           const int& host_udp_port_control,
                                           const std::string& client_ip,
                                           const DefaultScanRange& scan_range)
  : scan_range_(scan_range)
{
  const auto host_ip_number = inet_network(host_ip.c_str());
  if (static_cast<in_addr_t>(-1) == host_ip_number)
  {
    throw std::invalid_argument("Host IP invalid");
  }
  assert(sizeof(host_ip_number) == 4 && "host_ip_number has not the expected size");
  host_ip_ = static_cast<uint32_t>(host_ip_number);

  if (host_udp_port_data < std::numeric_limits<uint16_t>::min() ||
      host_udp_port_data > std::numeric_limits<uint16_t>::max())
  {
    throw std::out_of_range("Host UDP port out of range");
  }
  host_udp_port_data_ = htole16(static_cast<uint16_t>(host_udp_port_data));

  if (host_udp_port_control < std::numeric_limits<uint16_t>::min() ||
      host_udp_port_control > std::numeric_limits<uint16_t>::max())
  {
    throw std::out_of_range("Host UDP port out of range");
  }
  host_udp_port_control_ = htole16(static_cast<uint16_t>(host_udp_port_control));

  const auto client_ip_number = inet_network(client_ip.c_str());
  if (static_cast<in_addr_t>(-1) == client_ip_number)
  {
    throw std::invalid_argument("client IP invalid");
  }
  assert(sizeof(client_ip_number) == 4 && "client_ip_number has not the expected size");
  client_ip_ = static_cast<uint32_t>(client_ip_number);
}

uint32_t ScannerConfiguration::hostIp() const
{
  return host_ip_;
}

uint16_t ScannerConfiguration::hostUDPPortData() const
{
  return host_udp_port_data_;
}

uint16_t ScannerConfiguration::hostUDPPortControl() const
{
  return host_udp_port_control_;
}

uint32_t ScannerConfiguration::clientIp() const
{
  return client_ip_;
}

const DefaultScanRange& ScannerConfiguration::scanRange() const
{
  return scan_range_;
}

}  // namespace psen_scan_v2
