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
class MockCallbackHolder
{
public:
  MOCK_METHOD1(laserscan_callback, void(const LaserScan&));
};

static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr double START_ANGLE{ 0. };
static constexpr double END_ANGLE{ degreeToRadian(275.) };

static constexpr std::chrono::milliseconds DEFAULT_TIMEOUT{ 50 };

class ScannerTest : public testing::Test
{
protected:
  ScannerTest() : scanner_config_(HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, START_ANGLE, END_ANGLE)
  {
  }

  MockCallbackHolder mock_;
  ScannerConfiguration scanner_config_;
  LaserScanCallback laserscan_callback_{
    std::bind(&MockCallbackHolder::laserscan_callback, &mock_, std::placeholders::_1)
  };
};

typedef ScannerT<psen_scan_v2_test::ScannerControllerMock> MockedScanner;

TEST_F(ScannerTest, testConstructorInvalidLaserScanCallback)
{
  LaserScanCallback laserscan_callback;
  EXPECT_THROW(MockedScanner scanner(scanner_config_, laserscan_callback);, std::invalid_argument);
}

// TEST_F(ScannerTest, testConstructorSuccess)
// {
//   EXPECT_NO_THROW(ScannerT<ScannerControllerMock>());
// }

TEST_F(ScannerTest, testStart)
{
  MockedScanner scanner(scanner_config_, laserscan_callback_);
  std::promise<void> mocked_start_finished_barrier;
  EXPECT_CALL(scanner.scanner_controller_, start()).WillOnce(InvokeWithoutArgs([&mocked_start_finished_barrier]() {
    return mocked_start_finished_barrier.get_future();
  }));

  std::future<void> start_future = std::async(std::launch::async, [&scanner]() { scanner.start(); });

  EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::timeout) << "Scanner::start() already finished";
  mocked_start_finished_barrier.set_value();
  EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
}

TEST_F(ScannerTest, testStop)
{
  MockedScanner scanner(scanner_config_, laserscan_callback_);
  std::promise<void> mocked_stop_finished_barrier;
  EXPECT_CALL(scanner.scanner_controller_, stop()).WillOnce(InvokeWithoutArgs([&mocked_stop_finished_barrier]() {
    return mocked_stop_finished_barrier.get_future();
  }));

  std::future<void> stop_future = std::async(std::launch::async, [&scanner]() { scanner.stop(); });

  EXPECT_EQ(stop_future.wait_for(DEFAULT_TIMEOUT), std::future_status::timeout) << "Scanner::stop() already finished";
  mocked_stop_finished_barrier.set_value();
  EXPECT_EQ(stop_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::stop() not finished";
}
}  // namespace psen_scan_v2

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
