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
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/msg_decoder.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner_controller.h"
#include "psen_scan_v2/udp_client.h"

namespace psen_scan_v2
{
class LaserScanBuildFailure : public std::runtime_error
{
public:
  LaserScanBuildFailure(const std::string& msg = "Error while building laser scan");
};

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
   */
  ScannerT(const ScannerConfiguration& scanner_config);
  //! @brief Starts the scanner.
  void start();
  //! @brief Stops the scanner.
  void stop();
  //! @brief Fetches a complete laser scan from the scanner.
  LaserScan getCompleteScan();

private:
  SC scanner_controller_;

  friend class ScannerTest;
  FRIEND_TEST(ScannerTest, testConstructorSuccess);
  FRIEND_TEST(ScannerTest, testStart);
  FRIEND_TEST(ScannerTest, testStop);
  FRIEND_TEST(ScannerTest, testGetCompleteScan);
};

typedef ScannerT<> Scanner;

inline LaserScanBuildFailure::LaserScanBuildFailure(const std::string& msg) : std::runtime_error(msg)
{
}

template <typename SC>
ScannerT<SC>::ScannerT(const ScannerConfiguration& scanner_config) : scanner_controller_(scanner_config)
{
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

template <typename SC>
LaserScan ScannerT<SC>::getCompleteScan()
{
  // TODO: Add implementation in following stories
  throw LaserScanBuildFailure();
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_H
