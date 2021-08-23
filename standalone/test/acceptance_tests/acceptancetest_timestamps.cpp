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
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include <psen_scan_v2_standalone/core.h>

using namespace psen_scan_v2_standalone;

void laserScanCallback(const LaserScan& scan)
{
  PSENSCAN_INFO("scanCB()", "{}: {}", scan.getTimestamp(), scan.getScanCounter());
}

int main(int argc, char** argv)
{
  setLogLevel(CONSOLE_BRIDGE_LOG_INFO);

  bool fragmented_scans{ false };
  std::string sensor_ip{ "192.168.0.10" };
  util::TenthOfDegree angle_start{ -1374 };
  util::TenthOfDegree angle_end{ 1374 };

  if (const char* env_p = std::getenv("SENSOR_IP"))
  {
    sensor_ip = std::string(env_p);
  }

  int i = 1;
  while (i + 1 < argc)
  {
    const auto arg_name{ std::string(argv[i]) };
    const auto arg_value{ std::string(argv[i + 1]) };
    if (arg_name == "angle_start")
    {
      angle_start = util::TenthOfDegree(std::stoi(arg_value));
    }
    if (arg_name == "angle_end")
    {
      angle_end = util::TenthOfDegree(std::stoi(arg_value));
    }
    if (arg_name == "fragmented_scans")
    {
      std::istringstream(arg_value) >> std::boolalpha >> fragmented_scans;
    }
    i += 2;
  }

  ScannerConfigurationBuilder config_builder;
  config_builder.enableFragmentedScans(fragmented_scans)
      .scannerIp(sensor_ip)
      .scanRange(ScanRange{ angle_start + util::TenthOfDegree::fromRad(configuration::DEFAULT_X_AXIS_ROTATION),
                            angle_end + util::TenthOfDegree::fromRad(configuration::DEFAULT_X_AXIS_ROTATION) });
  ScannerV2 scanner(config_builder.build(), laserScanCallback);

  scanner.start();
  std::this_thread::sleep_for(std::chrono::seconds(10));
  scanner.stop();

  return 0;
}
