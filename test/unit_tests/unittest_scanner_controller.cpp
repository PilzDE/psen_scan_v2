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

#include <algorithm>
#include <math.h>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "psen_scan_v2/controller_state_machine.h"
#include "psen_scan_v2/mock_udp_client.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner_controller.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/stop_request.h"
#include "psen_scan_v2/laserscan_conversions.h"
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/udp_frame_dumps.h"
#include "psen_scan_v2/raw_data_array_conversion.h"
#include "psen_scan_v2/scan_range.h"

using namespace psen_scan_v2_test;

using ::testing::StrictMock;

namespace psen_scan_v2
{
static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr DefaultScanRange SCAN_RANGE{ TenthOfDegree(0), TenthOfDegree(2750) };

class MockCallbackHolder
{
public:
  MOCK_METHOD1(laserscan_callback, void(const LaserScan&));
};

static constexpr uint32_t OP_CODE_START{ 0x35 };
static constexpr uint32_t OP_CODE_STOP{ 0x36 };
static constexpr uint32_t OP_CODE_UNKNOWN{ 0x01 };
static constexpr uint32_t RES_CODE_ACCEPTED{ 0x00 };

class ScannerControllerTest : public ::testing::Test
{
protected:
  void sendStartReply();
  void sendStopReply();

protected:
  MockCallbackHolder mock_;
  ScannerConfiguration scanner_config_{ HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, SCAN_RANGE };

  LaserScanCallback laser_scan_callback_{
    std::bind(&MockCallbackHolder::laserscan_callback, &mock_, std::placeholders::_1)
  };

  ScannerControllerT<ControllerStateMachine, MockUdpClient> scanner_controller_{ scanner_config_,
                                                                                 laser_scan_callback_ };
};

void ScannerControllerTest::sendStartReply()
{
  ScannerReplyMsg msg(OP_CODE_START, RES_CODE_ACCEPTED);
  const auto data{ msg.toRawData() };
  MaxSizeRawData max_size_data;
  std::copy_n(data.begin(), data.size(), max_size_data.begin());

  scanner_controller_.control_udp_client_.handleNewData(max_size_data, max_size_data.size());
}

void ScannerControllerTest::sendStopReply()
{
  ScannerReplyMsg msg(OP_CODE_STOP, RES_CODE_ACCEPTED);
  const auto data{ msg.toRawData() };
  MaxSizeRawData max_size_data;
  std::copy_n(data.begin(), data.size(), max_size_data.begin());

  scanner_controller_.control_udp_client_.handleNewData(max_size_data, max_size_data.size());
}

TEST_F(ScannerControllerTest, testStart)
{
  using ::testing::_;
  using ::testing::InSequence;

  StartRequest start_request(scanner_config_, 0);

  {
    InSequence seq;
    EXPECT_CALL(scanner_controller_.control_udp_client_, startAsyncReceiving(_, _, _)).Times(1);
    EXPECT_CALL(scanner_controller_.data_udp_client_, startAsyncReceiving()).Times(1);
    EXPECT_CALL(scanner_controller_.control_udp_client_, write(start_request.toRawData())).Times(1);
  }

  auto start_future = scanner_controller_.start();
  sendStartReply();
  start_future.wait_for(std::chrono::seconds(0));
}

TEST_F(ScannerControllerTest, testStop)
{
  using ::testing::_;
  using ::testing::InSequence;

  StopRequest stop_request;

  {
    InSequence seq;
    EXPECT_CALL(scanner_controller_.control_udp_client_, startAsyncReceiving(_, _, _)).Times(1);
    EXPECT_CALL(scanner_controller_.control_udp_client_, write(stop_request.toRawData())).Times(1);
  }

  auto stop_future = scanner_controller_.stop();
  sendStopReply();
  stop_future.wait_for(std::chrono::seconds(0));
}

TEST_F(ScannerControllerTest, testHandleNewMonitoringFrame)
{
  UDPFrameTestDataWithoutIntensities test_data;
  const MaxSizeRawData raw_data = convertToMaxSizeRawData(test_data.hex_dump);
  const auto num_bytes = 2 * test_data.hex_dump.size();
  MonitoringFrameMsg frame{ MonitoringFrameMsg::fromRawData(raw_data, num_bytes) };
  LaserScan scan = toLaserScan(frame);

  EXPECT_CALL(mock_, laserscan_callback(scan)).Times(1);

  scanner_controller_.start();
  sendStartReply();
  scanner_controller_.data_udp_client_.handleNewData(raw_data, raw_data.size());
}

TEST_F(ScannerControllerTest, testHandleEmptyMonitoringFrame)
{
  using ::testing::_;

  UDPFrameTestDataWithoutMeasurementsAndIntensities test_data;
  MaxSizeRawData raw_data = convertToMaxSizeRawData(test_data.hex_dump);
  const auto num_bytes = 2 * test_data.hex_dump.size();
  MonitoringFrameMsg frame{ MonitoringFrameMsg::fromRawData(raw_data, num_bytes) };

  EXPECT_CALL(mock_, laserscan_callback(_)).Times(0);

  scanner_controller_.start();
  sendStartReply();
  scanner_controller_.data_udp_client_.handleNewData(raw_data, raw_data.size());
}

TEST_F(ScannerControllerTest, testConstructorInvalidLaserScanCallback)
{
  LaserScanCallback laserscan_callback;
  typedef ScannerControllerT<ControllerStateMachine, MockUdpClient> SCT;
  EXPECT_THROW(SCT scanner_controller_(scanner_config_, laserscan_callback);, std::invalid_argument);
}

}  // namespace psen_scan_v2

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
