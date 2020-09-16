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

using namespace psen_scan_v2;

namespace psen_scan_v2
{
static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr double START_ANGLE{ 0. };
static constexpr double END_ANGLE{ 275. * 2 * M_PI / 360. };

class ScannerControllerTest : public ::testing::Test
{
protected:
  ScannerControllerTest()
    : scanner_config_(HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, START_ANGLE, END_ANGLE)
    , scanner_controller_(scanner_config_)
  {
  }

protected:
  ScannerConfiguration scanner_config_;
  ScannerControllerT<psen_scan_v2_test::ControllerStateMachineMock, psen_scan_v2_test::MockUdpClient>
      scanner_controller_;
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

TEST_F(ScannerControllerTest, testStartRequestSending)
{
  using ::testing::_;
  using ::testing::Expectation;

  StartRequest start_request(scanner_config_, 0);

  Expectation control_udp_client_start_receiving =
      EXPECT_CALL(scanner_controller_.control_udp_client_, startSingleAsyncReceiving(_, _));
  Expectation data_udp_client_start_receiving =
      EXPECT_CALL(scanner_controller_.data_udp_client_, startAsyncReceiving(_));
  EXPECT_CALL(scanner_controller_.control_udp_client_, write(start_request.toRawData()))
      .After(control_udp_client_start_receiving, data_udp_client_start_receiving);

  scanner_controller_.sendStartRequest();
}

TEST_F(ScannerControllerTest, testStopRequestSending)
{
  using ::testing::_;
  using ::testing::Expectation;

  StopRequest stop_request;

  EXPECT_CALL(scanner_controller_.control_udp_client_, write(stop_request.toRawData())).Times(1);

  scanner_controller_.sendStopRequest();
}

TEST_F(ScannerControllerTest, testHandleStartReplyTimeout)
{
  using ::testing::_;
  using ::testing::Expectation;

  StopRequest stop_request;
  EXPECT_CALL(scanner_controller_.control_udp_client_, startSingleAsyncReceiving(_, _))
      .WillOnce(::testing::Invoke(
          [](const TimeoutHandler& timeout_handler, const std::chrono::high_resolution_clock::duration timeout) {
            timeout_handler("timeout!");
          }));
  scanner_controller_.sendStartRequest();
}

TEST_F(ScannerControllerTest, test_handle_error_no_throw)
{
  ASSERT_NO_THROW(scanner_controller_.handleError("Error Message."));
}

}  // namespace psen_scan_v2

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
