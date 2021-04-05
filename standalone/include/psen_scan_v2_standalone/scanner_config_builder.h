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
#include <stdexcept>
#include <string>

#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scan_range.h"
#include "psen_scan_v2_standalone/data_conversion_layer/angle_conversions.h"
#include "psen_scan_v2_standalone/util/ip_conversion.h"

namespace psen_scan_v2_standalone
{
/**
 * @brief Helper class to simplify/improve the construction of the
 * psen_scan_v2_standalone::ScannerConfiguration.
 */
class ScannerConfigurationBuilder
{
public:
  ScannerConfiguration build() const;

public:
  ScannerConfigurationBuilder& hostIP(const std::string&);
  ScannerConfigurationBuilder& hostDataPort(const int&);
  ScannerConfigurationBuilder& hostControlPort(const int&);
  ScannerConfigurationBuilder& scannerIp(const std::string&);
  ScannerConfigurationBuilder& scannerDataPort(const int&);
  ScannerConfigurationBuilder& scannerControlPort(const int&);
  ScannerConfigurationBuilder& scanRange(const ScanRange&);
  ScannerConfigurationBuilder& enableDiagnostics();
  ScannerConfigurationBuilder& enableFragmentedScans(const bool&);

private:
  static uint16_t convertPort(const int& port);

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

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::hostIP(const std::string& ip)
{
  if (ip != "" && ip != "auto")
  {
    config_.host_ip_ = util::convertIP(ip);
  }
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::hostDataPort(const int& port)
{
  config_.host_data_port_ = util::convertPort(port);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::hostControlPort(const int& port)
{
  config_.host_control_port_ = util::convertPort(port);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::scannerIp(const std::string& ip)
{
  config_.scanner_ip_ = util::convertIP(ip);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::scannerDataPort(const int& port)
{
  config_.scanner_data_port_ = util::convertPort(port);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::scannerControlPort(const int& port)
{
  config_.scanner_control_port_ = util::convertPort(port);
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::scanRange(const ScanRange& scan_range)
{
  config_.scan_range_ = scan_range;
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::enableDiagnostics()
{
  config_.diagnostics_enabled_ = true;
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::enableFragmentedScans(const bool& enable)
{
  config_.fragmented_scans_ = enable;
  return *this;
}
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_CONFIG_BUILDER_H
