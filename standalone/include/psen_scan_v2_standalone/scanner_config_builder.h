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

#ifndef PSEN_SCAN_V2_STANDALONE_SCANNER_CONFIG_BUILDER_H
#define PSEN_SCAN_V2_STANDALONE_SCANNER_CONFIG_BUILDER_H

#include <cmath>
#include <cassert>
#include <stdexcept>
#include <string>
#include <limits>

#include <arpa/inet.h>

#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scan_range.h"
#include "psen_scan_v2_standalone/angle_conversions.h"

namespace psen_scan_v2_standalone
{
namespace configuration
{
/**
 * @brief Helper class to simplify/improve the construction of the
 * psen_scan_v2_standalone::configuration::ScannerConfiguration.
 */
class ScannerConfigurationBuilder
{
public:
  configuration::ScannerConfiguration build() const;

public:
  ScannerConfigurationBuilder& hostIP(const std::string&);
  ScannerConfigurationBuilder& hostDataPort(const int&);
  ScannerConfigurationBuilder& hostControlPort(const int&);
  ScannerConfigurationBuilder& scannerIp(const std::string&);
  ScannerConfigurationBuilder& scannerDataPort(const int&);
  ScannerConfigurationBuilder& scannerControlPort(const int&);
  ScannerConfigurationBuilder& scanRange(const DefaultScanRange&);
  ScannerConfigurationBuilder& enableDiagnostics();

private:
  static uint16_t convertPort(const int& port);
  static uint32_t convertIP(const std::string& ip);

private:
  ScannerConfiguration config_;
};

inline ScannerConfiguration ScannerConfigurationBuilder::build() const
{
  if (!config_.isValid())
  {
    throw std::runtime_error("Scanner configuration not complete");
  }
  return config_;
}

uint32_t ScannerConfigurationBuilder::convertIP(const std::string& ip)
{
  const auto ip_number = inet_network(ip.c_str());
  if (static_cast<in_addr_t>(-1) == ip_number)
  {
    throw std::invalid_argument("IP invalid");
  }
  assert(sizeof(ip_number) == 4 && "ip_number has not the expected size");
  return static_cast<uint32_t>(ip_number);
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::hostIP(const std::string& ip)
{
  config_.host_ip_ = convertIP(ip);

  return *this;
}

inline uint16_t ScannerConfigurationBuilder::convertPort(const int& port)
{
  if (port < std::numeric_limits<uint16_t>::min() || port > std::numeric_limits<uint16_t>::max())
  {
    throw std::out_of_range("Port out of range");
  }
  return htole16(static_cast<uint16_t>(port));
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::hostDataPort(const int& port)
{
  config_.host_data_port_ = convertPort(port);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::hostControlPort(const int& port)
{
  config_.host_control_port_ = convertPort(port);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::scannerIp(const std::string& ip)
{
  config_.scanner_ip_ = convertIP(ip);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::scannerDataPort(const int& port)
{
  config_.scanner_data_port_ = convertPort(port);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::scannerControlPort(const int& port)
{
  config_.scanner_control_port_ = convertPort(port);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::scanRange(const DefaultScanRange& scan_range)
{
  config_.scan_range_ = scan_range;
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::enableDiagnostics()
{
  config_.diagnostics_enabled_ = true;
  return *this;
}
}  // namespace configuration
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_CONFIG_BUILDER_H
