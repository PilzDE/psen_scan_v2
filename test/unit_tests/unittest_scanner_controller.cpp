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
#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/scan_range.h"

using namespace psen_scan_v2_test;
using ::testing::_;
using ::testing::InSequence;

using ::testing::NiceMock;

namespace psen_scan_v2
{
static const std::string HOST_IP{ "127.0.0.1" };
static constexpr int HOST_UDP_PORT_DATA{ 50505 };
static constexpr int HOST_UDP_PORT_CONTROL{ 55055 };
static const std::string DEVICE_IP{ "127.0.0.100" };
static constexpr DefaultScanRange SCAN_RANGE{ TenthOfDegree(0), TenthOfDegree(2750) };
static constexpr uint32_t DEFAULT_START_REQUEST_SEQ_NUMBER{ 0 };

static constexpr unsigned int FUTURE_READY_TIMEOUT_SEC{ 0 };

template <typename T>
bool isFutureReady(const std::future<T>& future_obj)
{
  return future_obj.wait_for(std::chrono::seconds(FUTURE_READY_TIMEOUT_SEC)) == std::future_status::ready;
}

class MockCallbackHolder
{
public:
  MOCK_METHOD1(laserscan_callback, void(const LaserScan&));
};

class ScannerControllerTest : public ::testing::Test
{
public:
  void simulateStartReply();
  void simulateStopReply();
  void simulateMonitoringFrame(MonitoringFrameMsg& msg);
  void simulateUdpError(const std::string& msg);
  void simulateUdpTimeout(const std::string& msg);

public:
  MockCallbackHolder mock_;
  ScannerConfiguration scanner_config_{ HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, SCAN_RANGE };

  LaserScanCallback laser_scan_callback_{
    std::bind(&MockCallbackHolder::laserscan_callback, &mock_, std::placeholders::_1)
  };

  ScannerControllerT<ControllerStateMachine, NiceMock<MockUdpClient> > scanner_controller_{ scanner_config_,
                                                                                            laser_scan_callback_ };
};

#define EXPECT_START_LISTENING_FOR_CONTROL1(scanner_controller)                                                        \
  EXPECT_CALL(scanner_controller.control_udp_client_, startAsyncReceiving(_))
#define EXPECT_START_LISTENING_FOR_CONTROL3(scanner_controller)                                                        \
  EXPECT_CALL(scanner_controller.control_udp_client_, startAsyncReceiving(_, _, _))
#define EXPECT_START_LISTENING_FOR_DATA(scanner_controller)                                                            \
  EXPECT_CALL(scanner_controller.data_udp_client_, startAsyncReceiving())
#define EXPECT_REQUEST_SEND(scanner_controller, request)                                                               \
  EXPECT_CALL(scanner_controller.control_udp_client_, write(request.serialize()))

void ScannerControllerTest::simulateStartReply()
{
  scanner_controller_.control_udp_client_.sendStartReply();
}

void ScannerControllerTest::simulateStopReply()
{
  scanner_controller_.control_udp_client_.sendStopReply();
}

void ScannerControllerTest::simulateMonitoringFrame(MonitoringFrameMsg& msg)
{
  scanner_controller_.data_udp_client_.sendMonitoringFrame(msg);
}

void ScannerControllerTest::simulateUdpError(const std::string& msg)
{
  scanner_controller_.control_udp_client_.simulateError(msg);
}

void ScannerControllerTest::simulateUdpTimeout(const std::string& msg)
{
  scanner_controller_.control_udp_client_.simulateTimeout(msg);
}

TEST_F(ScannerControllerTest, testSuccessfulStartSequence)
{
  {
    InSequence seq;
    EXPECT_START_LISTENING_FOR_CONTROL1(scanner_controller_).Times(1);
    EXPECT_REQUEST_SEND(scanner_controller_, StartRequest(scanner_config_, DEFAULT_START_REQUEST_SEQ_NUMBER)).Times(1);
  }
  EXPECT_START_LISTENING_FOR_DATA(scanner_controller_).Times(1);

  auto start_future = scanner_controller_.start();
  simulateStartReply();
  EXPECT_TRUE(isFutureReady(start_future));
}

TEST_F(ScannerControllerTest, testReceivingMultipleStartReplies)
{
  MonitoringFrameMsg msg(TenthOfDegree(0), TenthOfDegree(275), 1, { 0.1, 20., 25, 10, 1., 2., 3. });
  const LaserScan scan{ toLaserScan(msg) };

  EXPECT_CALL(mock_, laserscan_callback(toLaserScan(msg))).Times(2);

  scanner_controller_.start();
  simulateStartReply();
  simulateMonitoringFrame(msg);
  simulateStartReply();
  simulateMonitoringFrame(msg);
}

TEST_F(ScannerControllerTest, testSuccessfulStopSequence)
{
  {
    InSequence seq;
    EXPECT_START_LISTENING_FOR_CONTROL3(scanner_controller_).Times(1);
    EXPECT_REQUEST_SEND(scanner_controller_, StopRequest()).Times(1);
  }
  auto stop_future = scanner_controller_.stop();
  simulateStopReply();
  EXPECT_TRUE(isFutureReady(stop_future));
}

TEST_F(ScannerControllerTest, testStopWhileWaitingForStartReply)
{
  {
    InSequence seq;
    EXPECT_REQUEST_SEND(scanner_controller_, StartRequest(scanner_config_, DEFAULT_START_REQUEST_SEQ_NUMBER)).Times(1);
    EXPECT_REQUEST_SEND(scanner_controller_, StopRequest()).Times(1);
  }

  scanner_controller_.sendStartRequest();
  auto stop_future = scanner_controller_.stop();
  simulateStopReply();
  EXPECT_TRUE(isFutureReady(stop_future));
}

TEST_F(ScannerControllerTest, testStopReplyTimeout)
{
  // Has no defined behaviour yet
  {
    InSequence seq;
    EXPECT_CALL(scanner_controller_.control_udp_client_, startAsyncReceiving(_, _, _)).Times(1);
    EXPECT_CALL(scanner_controller_.control_udp_client_, write(StopRequest().serialize())).Times(1);
  }

  scanner_controller_.stop();
  simulateUdpTimeout("Udp timeout");
  simulateStopReply();
}

TEST_F(ScannerControllerTest, testHandleMonitoringFrame)
{
  MonitoringFrameMsg msg(TenthOfDegree(0), TenthOfDegree(275), 1, { 0.1, 20., 25, 10, 1., 2., 3. });

  const LaserScan scan{ toLaserScan(msg) };

  EXPECT_CALL(mock_, laserscan_callback(scan)).Times(1);

  scanner_controller_.start();
  simulateStartReply();
  simulateMonitoringFrame(msg);
}

TEST_F(ScannerControllerTest, testHandleEmptyMonitoringFrame)
{
  MonitoringFrameMsg msg(TenthOfDegree(1), TenthOfDegree(2), 42, {});
  EXPECT_CALL(mock_, laserscan_callback(_)).Times(0);

  scanner_controller_.start();
  simulateStartReply();

  simulateMonitoringFrame(msg);
}

TEST_F(ScannerControllerTest, testHandleEarlyMonitoringFrame)
{
  EXPECT_CALL(mock_, laserscan_callback(_)).Times(0);

  scanner_controller_.start();

  MonitoringFrameMsg msg(TenthOfDegree(0), TenthOfDegree(275), 1, { 0.1, 20., 25, 10, 1., 2., 3. });
  simulateMonitoringFrame(msg);
}

TEST_F(ScannerControllerTest, testHandleLateMonitoringFrame)
{
  EXPECT_CALL(mock_, laserscan_callback(_)).Times(0);

  scanner_controller_.start();
  simulateStartReply();

  scanner_controller_.stop();

  MonitoringFrameMsg msg(TenthOfDegree(0), TenthOfDegree(275), 1, { 0.1, 20., 25, 10, 1., 2., 3. });
  simulateMonitoringFrame(msg);

  simulateStopReply();
}

TEST_F(ScannerControllerTest, testHandleError)
{
  simulateUdpError("Udp error");  // only for coverage for now
}

TEST_F(ScannerControllerTest, testConstructorInvalidLaserScanCallback)
{
  EXPECT_THROW(({
                 ScannerControllerT<ControllerStateMachine, MockUdpClient> scanner_controller_(scanner_config_,
                                                                                               LaserScanCallback());
               }),
               std::invalid_argument);
}

}  // namespace psen_scan_v2

int main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
