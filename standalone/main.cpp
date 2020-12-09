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

#include <string>
#include <iostream>

#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner_config_builder.h"
#include "psen_scan_v2/scanner_v2.h"
#include "psen_scan_v2/scan_range.h"

using namespace psen_scan_v2;

/*
 * In this section we declare all necessary configuration parameters
 */
const std::string HOST_IP{ "192.168.0.50" };
const std::string SCANNER_IP{ "192.168.0.10" };
const TenthOfDegree ANGLE_START{ degreeToTenthDegree(0) };
const TenthOfDegree ANGLE_END{ degreeToTenthDegree(275) };

/*
 * This function is used as a callback every time a new laserscan is received.
 */
void laserScanCallback(const LaserScan& scan)
{
  LaserScan::MeasurementData measures = scan.getMeasurements();

  for(auto it = measures.cbegin(); it < measures.cend(); ++it)
  {
    std::cout << *it;
  }
  std::cout << std::endl;
}

int main(int argc, char** argv)
{
  DefaultScanRange scan_range{ ANGLE_START, ANGLE_END };

  ScannerConfigurationBuilder config_builder;
  config_builder.hostIP(HOST_IP)
      .scannerIp(SCANNER_IP)
      .scanRange(scan_range)
      .enableDiagnostics();

  ScannerConfiguration scanner_configuration{ config_builder.build() };

  ScannerV2 scanner(scanner_configuration, laserScanCallback);
  scanner.start();

  while(1);

  return 0;
}
