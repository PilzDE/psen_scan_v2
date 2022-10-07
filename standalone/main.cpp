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

#include <chrono>
#include <string>
#include <thread>

#include <psen_scan_v2_standalone/core.h>

using namespace psen_scan_v2_standalone;

/*
 * In this section we declare all necessary configuration parameters
 */
const std::string SCANNER_IP{ "192.168.0.10" };
// Start- and end-angle have been configured to be in the middle of the scan range.
const util::TenthOfDegree ANGLE_START{ data_conversion_layer::degreeToTenthDegree(137) };
const util::TenthOfDegree ANGLE_END{ data_conversion_layer::degreeToTenthDegree(138) };

/*
 * This function is used as a callback every time a new laserscan is received. It is skipped for Scans that contain no
 * measurement data. This can happen with smaller scan ranges.
 */
void laserScanCallback(const LaserScan& scan)
{
  // Other data fields are listed on
  // https://docs.ros.org/en/melodic/api/psen_scan_v2/html/classpsen__scan__v2__standalone_1_1LaserScan.html
  PSENSCAN_INFO_THROTTLE(1 /* sec */, "laserScanCallback()", "Ranges {}", util::formatRange(scan.measurements()));
}

int main(int argc, char** argv)
{
  setLogLevel(CONSOLE_BRIDGE_LOG_INFO);

  // Available configuration options are listed on
  // http://docs.ros.org/en/melodic/api/psen_scan_v2/html/classpsen__scan__v2__standalone_1_1ScannerConfigurationBuilder.html
  ScannerV2 scanner(ScannerConfigurationBuilder(SCANNER_IP).scanRange(ScanRange{ ANGLE_START, ANGLE_END }),
                    laserScanCallback);

  scanner.start();
  std::this_thread::sleep_for(std::chrono::seconds(10));
  scanner.stop();

  return 0;
}
