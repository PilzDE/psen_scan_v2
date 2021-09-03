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

#ifndef PSEN_SCAN_V2_TEST_TEST_DATA_ASSEMBLER_H
#define PSEN_SCAN_V2_TEST_TEST_DATA_ASSEMBLER_H

#include <cstdint>
#include <memory>
#include <string>

#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scanner_interface.h"

#include "psen_scan_v2/test_data.h"
#include "psen_scan_v2/udp_data_reader.h"

namespace psen_scan_v2_test
{
using namespace psen_scan_v2_standalone;

class TestDataAssembler
{
public:
  static std::unique_ptr<TestData> assemble(const ScannerConfiguration& scanner_config,
                                            const int64_t scanner_run_duration_sec,
                                            const std::string& udp_data_filename,
                                            const uint16_t udp_port);

private:
  static void runScanner(const ScannerConfiguration& scanner_config,
                         const int64_t scanner_run_duration_sec,
                         const IScanner::LaserScanCallback& laserscan_callback);
  static void extractDataFromScan(TestData& test_data, const LaserScan& scan);

  static void addUdpData(TestData& test_data, const UdpData& udp_data);

  static void removeIncompleteData(TestData& test_data);
  static void sortWithRespectToScanCounter(TestData& test_data);
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_TEST_DATA_ASSEMBLER_H
