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
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/scanner.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner_controller.h"
#include "psen_scan_v2/scanner_controller_mock.h"
#include "psen_scan_v2/scan_range.h"

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
static constexpr DefaultScanRange SCAN_RANGE{ TenthOfDegree(0), TenthOfDegree(2750) };

class ScannerTest : public testing::Test
{
protected:
  ScannerTest() : scanner_config_(HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, SCAN_RANGE)
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

TEST_F(ScannerTest, testConstructorSuccess)
{
  EXPECT_NO_THROW(MockedScanner scanner(scanner_config_, laserscan_callback_));
}

TEST_F(ScannerTest, testInvokeLaserScanCallback)
{
  LaserScan laser_scan_fake(TenthOfDegree(1), TenthOfDegree(3), TenthOfDegree(5));
  laser_scan_fake.getMeasurements().push_back(1);

  MockedScanner scanner(scanner_config_, laserscan_callback_);
  EXPECT_CALL(mock_, laserscan_callback(laser_scan_fake)).Times(1);

  scanner.scanner_controller_.invokeLaserScanCallback(laser_scan_fake);
}
}  // namespace psen_scan_v2

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
