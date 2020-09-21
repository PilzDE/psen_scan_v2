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

#include <future>
#include <thread>
#include <chrono>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2/angle_conversions.h"
#include "psen_scan_v2/scanner.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner_controller.h"
#include "psen_scan_v2/scanner_controller_mock.h"

using namespace psen_scan_v2;

using namespace ::testing;

namespace psen_scan_v2
{
static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr double START_ANGLE{ 0. };
static constexpr double END_ANGLE{ degreeToRad(275.) };

class ScannerTest : public testing::Test
{
protected:
  ScannerTest() : scanner_config_(HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, START_ANGLE, END_ANGLE)
  {
  }

  ScannerConfiguration scanner_config_;
};
// TEST_F(ScannerTest, testConstructorSuccess)
// {
//   EXPECT_NO_THROW(ScannerT<ScannerControllerMock>());
// }

typedef ScannerT<psen_scan_v2_test::ScannerControllerMock> MockedScanner;

TEST_F(ScannerTest, testStart)
{
  MockedScanner scanner(scanner_config_);
  EXPECT_CALL(scanner.scanner_controller_, start()).Times(1);
  scanner.start();
}

TEST_F(ScannerTest, testStop)
{
  MockedScanner scanner(scanner_config_);
  std::promise<void> mocked_stop_finished_barrier;
  EXPECT_CALL(scanner.scanner_controller_, stop()).WillOnce(InvokeWithoutArgs([&mocked_stop_finished_barrier]() {
    return mocked_stop_finished_barrier.get_future();
  }));

  std::promise<void> stop_func_finished_barrier;
  std::future<void> stop_func_finished_barrier_future = stop_func_finished_barrier.get_future();
  std::future<void> stop_future = std::async(std::launch::async, [&scanner, &stop_func_finished_barrier]() {
    scanner.stop();
    stop_func_finished_barrier.set_value();
  });

  EXPECT_EQ(stop_func_finished_barrier_future.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout)
      << "Scanner::stop() function has finished too early";
  mocked_stop_finished_barrier.set_value();
  EXPECT_EQ(stop_func_finished_barrier_future.wait_for(std::chrono::milliseconds(50)), std::future_status::ready)
      << "Scanner::stop() function has not finished";
}

TEST_F(ScannerTest, testGetCompleteScan)
{
  MockedScanner scanner(scanner_config_);
  EXPECT_THROW(scanner.getCompleteScan(), LaserScanBuildFailure);
}

}  // namespace psen_scan_v2

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
