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
  ScannerConfigurationBuilder& scanResolution(const util::TenthOfDegree&);
  ScannerConfigurationBuilder& enableDiagnostics(const bool&);
  ScannerConfigurationBuilder& enableIntensities(const bool&);
  ScannerConfigurationBuilder& enableFragmentedScans(const bool&);

private:
  static uint16_t convertPort(const int& port);

private:
  ScannerConfiguration config_;
};

inline ScannerConfiguration ScannerConfigurationBuilder::build() const
{
  if (!config_.isComplete())
  {
    throw std::runtime_error("Scanner configuration not complete");
  }
  else if (!config_.isValid())
  {
    throw std::invalid_argument("Scanner configuration not valid");
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

inline ScannerConfigurationBuilder&
ScannerConfigurationBuilder::scanResolution(const util::TenthOfDegree& scan_resolution)
{
  if (scan_resolution < util::TenthOfDegree(1) || scan_resolution > util::TenthOfDegree(100))
  {
    throw std::invalid_argument("Scan resolution has to be between 0.1 and 10 degrees.");
  }
  config_.scan_resolution_ = scan_resolution;
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::enableDiagnostics(const bool& enable = true)
{
  config_.diagnostics_enabled_ = enable;
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::enableIntensities(const bool& enable = true)
{
  config_.intensities_enabled_ = enable;
  return *this;
}

inline ScannerConfigurationBuilder& ScannerConfigurationBuilder::enableFragmentedScans(const bool& enable = true)
{
  config_.fragmented_scans_ = enable;
  return *this;
}
}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_CONFIG_BUILDER_H
