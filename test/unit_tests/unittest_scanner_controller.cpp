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

#include <math.h>
#include <memory>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "psen_scan_v2/state_machine_controller_mock.h"
#include "psen_scan_v2/mock_udp_client.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner_controller.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/stop_request.h"
#include "psen_scan_v2/laserscan_conversions.h"
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/udp_frame_dumps.h"
#include "psen_scan_v2/raw_data_array_conversion.h"

using namespace psen_scan_v2_test;

namespace psen_scan_v2
{
static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr double START_ANGLE{ 0. };
static constexpr double END_ANGLE{ 275. * 2 * M_PI / 360. };

class MockCallbackHolder
{
public:
  MOCK_METHOD1(laserscan_callback, void(const LaserScan&));
};

class ScannerControllerTest : public ::testing::Test
{
protected:
  MockCallbackHolder mock_;
  ScannerConfiguration scanner_config_{ HOST_IP,   HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL,
                                        DEVICE_IP, START_ANGLE,        END_ANGLE };

  LaserScanCallback laser_scan_callback_{
    std::bind(&MockCallbackHolder::laserscan_callback, &mock_, std::placeholders::_1)
  };

  ScannerControllerT<psen_scan_v2_test::ControllerStateMachineMock, psen_scan_v2_test::MockUdpClient>
      scanner_controller_{ scanner_config_, laser_scan_callback_ };
};

TEST_F(ScannerControllerTest, testStartRequestEvent)
{
  EXPECT_CALL(scanner_controller_.state_machine_, processStartRequestEvent()).Times(1);

  scanner_controller_.start();
}

TEST_F(ScannerControllerTest, testStopRequestEvent)
{
  EXPECT_CALL(scanner_controller_.state_machine_, processStopRequestEvent()).Times(1);

  scanner_controller_.stop();
}

TEST_F(ScannerControllerTest, testStopRequestEventWithFutureUsage)
{
  EXPECT_CALL(scanner_controller_.state_machine_, processStopRequestEvent()).Times(1);

  std::future<void> stop_future = scanner_controller_.stop();

  scanner_controller_.state_machine_.stopped_cb_();  // TODO needs to be real async?

  stop_future.wait();
}

TEST_F(ScannerControllerTest, testStartRequestSending)
{
  using ::testing::_;
  using ::testing::Expectation;

  StartRequest start_request(scanner_config_, 0);

  Expectation control_udp_client_start_receiving =
      EXPECT_CALL(scanner_controller_.control_udp_client_, startAsyncReceiving(_, _, _));
  Expectation data_udp_client_start_receiving =
      EXPECT_CALL(scanner_controller_.data_udp_client_, startAsyncReceiving());
  EXPECT_CALL(scanner_controller_.control_udp_client_, write(start_request.toRawData()))
      .After(control_udp_client_start_receiving, data_udp_client_start_receiving);

  scanner_controller_.sendStartRequest();
}

TEST_F(ScannerControllerTest, testStopRequestSending)
{
  using ::testing::_;
  using ::testing::Expectation;

  StopRequest stop_request;

  EXPECT_CALL(scanner_controller_.control_udp_client_, startAsyncReceiving(_, _, _)).Times(1);
  EXPECT_CALL(scanner_controller_.control_udp_client_, write(stop_request.toRawData())).Times(1);

  scanner_controller_.sendStopRequest();
}

TEST_F(ScannerControllerTest, testHandleStartReplyTimeout)
{
  using ::testing::_;
  using ::testing::Expectation;

  EXPECT_CALL(scanner_controller_.control_udp_client_, startAsyncReceiving(_, _, _))
      .WillOnce(::testing::Invoke(
          [](const ReceiveMode& modi,
             const TimeoutHandler& timeout_handler,
             const std::chrono::high_resolution_clock::duration timeout) { timeout_handler("timeout!"); }));
  scanner_controller_.sendStartRequest();
}

TEST_F(ScannerControllerTest, testHandleStopReplyTimeout)
{
  using ::testing::_;
  using ::testing::Expectation;

  EXPECT_CALL(scanner_controller_.control_udp_client_, startAsyncReceiving(_, _, _))
      .WillOnce(::testing::Invoke(
          [](const ReceiveMode& modi,
             const TimeoutHandler& timeout_handler,
             const std::chrono::high_resolution_clock::duration timeout) { timeout_handler("timeout!"); }));
  scanner_controller_.sendStopRequest();
}

TEST_F(ScannerControllerTest, test_handle_error_no_throw)
{
  ASSERT_NO_THROW(scanner_controller_.handleError("Error Message."));
}

TEST_F(ScannerControllerTest, testHandleNewMonitoringFrame)
{
  UDPFrameTestDataWithoutIntensities test_data;
  MaxSizeRawData data = convertToMaxSizeRawData(test_data.hex_dump);
  MonitoringFrameMsg frame{ MonitoringFrameMsg::fromRawData(data) };
  LaserScan scan = toLaserScan(frame);

  EXPECT_CALL(scanner_controller_.state_machine_, processMonitoringFrameReceivedEvent()).Times(1);
  EXPECT_CALL(mock_, laserscan_callback(scan)).Times(1);

  scanner_controller_.handleNewMonitoringFrame(data, data.size());
}

TEST_F(ScannerControllerTest, testConstructorInvalidLaserScanCallback)
{
  LaserScanCallback laserscan_callback;
  typedef ScannerControllerT<psen_scan_v2_test::ControllerStateMachineMock, psen_scan_v2_test::MockUdpClient> SCT;
  EXPECT_THROW(SCT scanner_controller_(scanner_config_, laserscan_callback);, std::invalid_argument);
}

}  // namespace psen_scan_v2

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
