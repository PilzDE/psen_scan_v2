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

#ifndef PSEN_SCAN_V2_STANDALONE_SCANNER_H
#define PSEN_SCAN_V2_STANDALONE_SCANNER_H

#include <stdexcept>
#include <future>
#include <functional>

#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"

namespace psen_scan_v2_standalone
{
/**
 * @brief This is the API definition for external interaction with the scanner driver.
 *
 * This interface allows to:
 * - Set a configuration for the scanner on startup
 * - Define a callback for incoming scans
 * - Start and stop the communication with the scanner
 *
 * @see protocol_layer::LaserScanCallback
 * @see ScannerConfiguration
 * @see ScannerV2
 */
class IScanner
{
public:
  //! @brief Represents the user-provided callback for processing incoming scan data.
  using LaserScanCallback = std::function<void(const LaserScan&)>;

public:
  IScanner(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_callback);
  virtual ~IScanner() = default;

public:
  //! @brief Starts the scanner.
  virtual std::future<void> start() = 0;
  //! @brief Stops the scanner.
  virtual std::future<void> stop() = 0;

protected:
  const ScannerConfiguration& getConfig() const;
  const LaserScanCallback& getLaserScanCallback() const;

private:
  const ScannerConfiguration config_;
  const LaserScanCallback laser_scan_callback_;
};

inline IScanner::IScanner(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_callback)
  : config_(scanner_config), laser_scan_callback_(laser_scan_callback)
{
  if (!laser_scan_callback)
  {
    throw std::invalid_argument("Laserscan-callback must not be null");
  }
}

inline const ScannerConfiguration& IScanner::getConfig() const
{
  return config_;
}

inline const IScanner::LaserScanCallback& IScanner::getLaserScanCallback() const
{
  return laser_scan_callback_;
}

}  // namespace psen_scan_v2_standalone

#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_H
