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

#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/util/logging.h"
#include "psen_scan_v2_standalone/scan_range.h"

namespace psen_scan_v2_standalone
{
/**
 * @brief Higher level data type storing the configuration details of the scanner like scanner IP, port,
 * scan range, etc.
 *
 * If no host IP is set the driver will use the IP of the local machine.
 *
 * @see ScanRange
 */
class ScannerConfiguration
{
private:
  ScannerConfiguration() = default;

public:
  boost::optional<uint32_t> hostIp() const;
  uint16_t hostUDPPortData() const;
  uint16_t hostUDPPortControl() const;

  uint32_t clientIp() const;
  uint16_t scannerDataPort() const;
  uint16_t scannerControlPort() const;

  const ScanRange& scanRange() const;
  const util::TenthOfDegree& scanResolution() const;

  bool diagnosticsEnabled() const;
  bool intensitiesEnabled() const;

  bool fragmentedScansEnabled() const;

  /*! deprecated: use void hostIp(const uint32_t& host_ip) instead */
  [[deprecated("use void hostIp(const uint32_t& host_ip) instead")]] void setHostIp(const uint32_t& host_ip);
  void hostIp(const uint32_t& host_ip);

private:
  friend class ScannerConfigurationBuilder;

private:
  bool isComplete() const;
  bool isValid() const;

private:
  boost::optional<uint32_t> host_ip_;
  uint16_t host_data_port_{ configuration::DATA_PORT_OF_HOST_DEVICE };
  uint16_t host_control_port_{ configuration::CONTROL_PORT_OF_HOST_DEVICE };

  boost::optional<uint32_t> scanner_ip_;
  uint16_t scanner_data_port_{ configuration::DATA_PORT_OF_SCANNER_DEVICE };
  uint16_t scanner_control_port_{ configuration::CONTROL_PORT_OF_SCANNER_DEVICE };

  boost::optional<ScanRange> scan_range_{};
  util::TenthOfDegree scan_resolution_{ data_conversion_layer::radToTenthDegree(
      configuration::DEFAULT_SCAN_ANGLE_RESOLUTION) };
  bool diagnostics_enabled_{ configuration::DIAGNOSTICS };
  bool intensities_enabled_{ configuration::INTENSITIES };
  bool fragmented_scans_{ configuration::FRAGMENTED_SCANS };
};

inline bool ScannerConfiguration::isComplete() const
{
  return scanner_ip_ && scan_range_;
}

inline bool ScannerConfiguration::isValid() const
{
  if (intensities_enabled_ && scan_resolution_ < util::TenthOfDegree(2u))
  {
    PSENSCAN_ERROR("ScannerConfiguration", "Requires a resolution of min: 0.2 degree when intensities are enabled");
    return false;
  }
  return true;
}

inline boost::optional<uint32_t> ScannerConfiguration::hostIp() const
{
  return host_ip_;
}

inline uint16_t ScannerConfiguration::hostUDPPortData() const
{
  return host_data_port_;
}

inline uint16_t ScannerConfiguration::hostUDPPortControl() const
{
  return host_control_port_;
}

inline uint32_t ScannerConfiguration::clientIp() const
{
  return *scanner_ip_;
}

inline uint16_t ScannerConfiguration::scannerDataPort() const
{
  return scanner_data_port_;
}

inline uint16_t ScannerConfiguration::scannerControlPort() const
{
  return scanner_control_port_;
}

inline const ScanRange& ScannerConfiguration::scanRange() const
{
  return *scan_range_;
}

inline const util::TenthOfDegree& ScannerConfiguration::scanResolution() const
{
  return scan_resolution_;
}

inline bool ScannerConfiguration::diagnosticsEnabled() const
{
  return diagnostics_enabled_;
}

inline bool ScannerConfiguration::intensitiesEnabled() const
{
  return intensities_enabled_;
}

inline bool ScannerConfiguration::fragmentedScansEnabled() const
{
  return fragmented_scans_;
}

inline void ScannerConfiguration::hostIp(const uint32_t& host_ip)
{
  host_ip_ = host_ip;
}

inline void ScannerConfiguration::setHostIp(const uint32_t& host_ip)
{
  host_ip_ = host_ip;
}

}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_CONFIGURATION_H
