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

#ifndef PSEN_SCAN_V2_SCANNER_H
#define PSEN_SCAN_V2_SCANNER_H

#include <functional>
#include <memory>
#include <stdexcept>

#include <gtest/gtest_prod.h>

#include "psen_scan_v2/controller_state_machine.h"
#include "psen_scan_v2/function_pointers.h"
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/msg_decoder.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner_controller.h"
#include "psen_scan_v2/udp_client.h"

namespace psen_scan_v2
{
/**
 * @brief API to control and to fetch measurements from the scanner.
 */
template <typename SC = ScannerController>
class ScannerT
{
public:
  /**
   * @brief Constructor.
   *
   * @param scanner_config Configuration details of the scanner.
   * @param laser_scan_callback Callback for processing complete laser scans.
   */
  ScannerT(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_callback);
  //! @brief Starts the scanner.
  void start();
  //! @brief Stops the scanner.
  void stop();

private:
  SC scanner_controller_;

  friend class ScannerTest;
  FRIEND_TEST(ScannerTest, testConstructorSuccess);
  FRIEND_TEST(ScannerTest, testStart);
  FRIEND_TEST(ScannerTest, testStop);
  FRIEND_TEST(ScannerTest, testInvokeLaserScanCallback);
};

typedef ScannerT<> Scanner;

template <typename SC>
ScannerT<SC>::ScannerT(const ScannerConfiguration& scanner_config, const LaserScanCallback& laser_scan_callback)
  : scanner_controller_(scanner_config, laser_scan_callback)
{
  if (!laser_scan_callback)
  {
    throw std::invalid_argument("Invalid laserscanner callback registered!");
  }
}

template <typename SC>
void ScannerT<SC>::start()
{
  std::future<void> start_future = scanner_controller_.start();
  PSENSCAN_INFO("Scanner", "Waiting for scanner to start...");
  start_future.wait();
  PSENSCAN_INFO("Scanner", "Scanner has started.");
}

template <typename SC>
void ScannerT<SC>::stop()
{
  std::future<void> stop_future = scanner_controller_.stop();
  PSENSCAN_INFO("Scanner", "Waiting for scanner to stop...");
  stop_future.wait();
  PSENSCAN_INFO("Scanner", "Scanner has stopped.");
}
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_H
