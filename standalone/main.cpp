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

#include <chrono>
#include <sstream>
#include <string>
#include <thread>

#include <psen_scan_v2_standalone/core.h>
#include <psen_scan_v2_standalone/logging.h>

using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_standalone::constants;

/*
 * In this section we declare all necessary configuration parameters
 */
const std::string HOST_IP{ "192.168.0.50" };
const std::string SCANNER_IP{ "192.168.0.10" };
const unsigned short HOST_UDP_DATA_PORT{ 55115 };
const unsigned short HOST_UDP_CONTROL_PORT{ 55116 };
// Start- and end-angle have been configured to be in the middle of the scan range.
const TenthOfDegree ANGLE_START{ degreeToTenthDegree(137) };
const TenthOfDegree ANGLE_END{ degreeToTenthDegree(138) };

/*
 * This function is used as a callback every time a new laserscan is received.
 */
void laserScanCallback(const LaserScan& scan)
{
  const LaserScan::MeasurementData& measures = scan.getMeasurements();

  std::stringstream strstr;
  for (auto it = measures.cbegin(); it < measures.cend(); ++it)
  {
    strstr << " " << *it;
  }
  PSENSCAN_INFO_THROTTLE(1 /* sec */, "laserScanCallback()", strstr.str());
}

int main(int argc, char** argv)
{
  DefaultScanRange scan_range{ ANGLE_START, ANGLE_END };

  ScannerConfigurationBuilder config_builder;
  config_builder.hostIP(HOST_IP)
      .hostDataPort(HOST_UDP_DATA_PORT)
      .hostControlPort(HOST_UDP_CONTROL_PORT)
      .scannerIp(SCANNER_IP)
      .scannerDataPort(DATA_PORT_OF_SCANNER_DEVICE)
      .scannerControlPort(CONTROL_PORT_OF_SCANNER_DEVICE)
      .scanRange(scan_range)
      .enableDiagnostics();

  ScannerV2 scanner(config_builder.build(), laserScanCallback);
  scanner.start();

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
