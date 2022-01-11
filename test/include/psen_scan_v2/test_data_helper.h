// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_TEST_TEST_DATA_HELPER_H
#define PSEN_SCAN_V2_TEST_TEST_DATA_HELPER_H

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "psen_scan_v2_standalone/core.h"
#include "psen_scan_v2_standalone/util/timestamp.h"

#include "psen_scan_v2/test_data.h"
#include "psen_scan_v2/udp_data_helper.h"

namespace psen_scan_v2_test
{
using namespace std::chrono_literals;
using namespace psen_scan_v2_standalone;

namespace test_data
{
static constexpr uint16_t FIRST_FROM_THETA{ 2500 };  // change to 1 after scan counter fix in firmware

static int64_t secToNSec(const double& sec)
{
  double nsec{ sec * 1000000000.0 };
  return static_cast<int64_t>(nsec >= 0 ? nsec + 0.5 : nsec - 0.5);
}

static void runScanner(const ScannerConfiguration& scanner_config,
                       const int64_t scanner_run_duration_sec,
                       const IScanner::LaserScanCallback& laserscan_callback)
{
  ScannerV2 scanner(scanner_config, laserscan_callback);
  scanner.start();
  std::this_thread::sleep_for(std::chrono::seconds(scanner_run_duration_sec));
  auto stop_future = scanner.stop();
  if (stop_future.wait_for(3s) != std::future_status::ready)
  {
    throw std::runtime_error("Timeout while waiting for the scanner to stop.");
  }
  stop_future.get();  // catch exceptions of scanner stop
}

static void addUdpData(TestData& test_data, const udp_data::UdpData& udp_data)
{
  for (const auto& udp_datum : udp_data)
  {
    if (udp_datum.from_theta_ == FIRST_FROM_THETA)
    {
      const auto it = std::find_if(test_data.begin(), test_data.end(), [&udp_datum](const auto& test_datum) {
        return test_datum.scanCounter() == udp_datum.scan_counter_;
      });
      if (it != test_data.end())
      {
        it->firstFrameTime(secToNSec(udp_datum.timestamp_sec_));
      }
    }
  }
}

static void removeIncompleteData(TestData& test_data)
{
  test_data.erase(
      std::remove_if(test_data.begin(), test_data.end(), [](const auto& datum) { return !datum.isComplete(); }),
      test_data.end());
}

static void sortWithRespectToScanCounter(TestData& test_data)
{
  std::sort(test_data.begin(), test_data.end(), [](const auto& datum1, const auto& datum2) {
    return datum1.scanCounter() < datum2.scanCounter();
  });
}

static void extractDataFromScan(TestData& test_data, const LaserScan& scan)
{
  test_data.push_back(TestDatum(scan.scanCounter(), scan.timestamp(), util::getCurrentTime()));
}

static std::unique_ptr<TestData> assemble(const ScannerConfiguration& scanner_config,
                                          const int64_t scanner_run_duration_sec,
                                          const std::string& udp_data_filename,
                                          const uint16_t udp_port)
{
  std::unique_ptr<TestData> test_data_ptr{ new TestData() };
  runScanner(scanner_config, scanner_run_duration_sec, [&test_data_ptr](const LaserScan& scan) {
    extractDataFromScan(*test_data_ptr, scan);
  });
  PSENSCAN_INFO("test_data::assemble()", "Extracted data from {} laserscan callbacks.", test_data_ptr->size());

  udp_data::UdpData udp_data;
  udp_data::read(udp_data_filename, udp_port, udp_data);
  addUdpData(*test_data_ptr, udp_data);

  removeIncompleteData(*test_data_ptr);
  sortWithRespectToScanCounter(*test_data_ptr);
  return test_data_ptr;
}

}  // namespace test_data
}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_TEST_DATA_HELPER_H
