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
#include "psen_scan_v2/udp_frame_dumps.h"
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
  void sendStartReply();
  void sendStopReply();
  template <typename TestData>
  void sendMonitoringFrame(const TestData& test_data);
  void simulateUdpError(const std::string& msg);
  void simulateUdpTimeout(const std::string& msg);
  template <typename TestData>
  LaserScan testDataToLaserScan(const TestData& test_data);

public:
  MockCallbackHolder mock_;
  ScannerConfiguration scanner_config_{ HOST_IP, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, DEVICE_IP, SCAN_RANGE };

  LaserScanCallback laser_scan_callback_{
    std::bind(&MockCallbackHolder::laserscan_callback, &mock_, std::placeholders::_1)
  };

  ScannerControllerT<ControllerStateMachine, NiceMock<MockUdpClient> > scanner_controller_{ scanner_config_,
                                                                                            laser_scan_callback_ };
};

#define EXPECT_START_LISTENING_FOR_CONTROL(scanner_controller)                                                         \
  EXPECT_CALL(scanner_controller.control_udp_client_, startAsyncReceiving(_, _, _))
#define EXPECT_START_LISTENING_FOR_DATA(scanner_controller)                                                            \
  EXPECT_CALL(scanner_controller.data_udp_client_, startAsyncReceiving())
#define EXPECT_REQUEST_SEND(scanner_controller, request)                                                               \
  EXPECT_CALL(scanner_controller.control_udp_client_, write(request.serialize()))

void ScannerControllerTest::sendStartReply()
{
  scanner_controller_.control_udp_client_.sendStartReply();
}

void ScannerControllerTest::sendStopReply()
{
  scanner_controller_.control_udp_client_.sendStopReply();
}

template <typename TestData>
void ScannerControllerTest::sendMonitoringFrame(const TestData& test_data)
{
  scanner_controller_.data_udp_client_.sendMonitoringFrame(test_data);
}

void ScannerControllerTest::simulateUdpError(const std::string& msg)
{
  scanner_controller_.control_udp_client_.simulateError(msg);
}

void ScannerControllerTest::simulateUdpTimeout(const std::string& msg)
{
  scanner_controller_.control_udp_client_.simulateTimeout(msg);
}

template <typename TestData>
LaserScan ScannerControllerTest::testDataToLaserScan(const TestData& test_data)
{
  const MaxSizeRawData raw_data = convertToMaxSizeRawData(test_data.hex_dump);
  const auto num_bytes = 2 * test_data.hex_dump.size();
  const MonitoringFrameMsg frame{ MonitoringFrameMsg::deserialize(raw_data, num_bytes) };
  return toLaserScan(frame);
}

TEST_F(ScannerControllerTest, successfulStartSequence)
{
  {
    InSequence seq;
    EXPECT_START_LISTENING_FOR_CONTROL(scanner_controller_).Times(1);
    EXPECT_REQUEST_SEND(scanner_controller_, StartRequest(scanner_config_, DEFAULT_START_REQUEST_SEQ_NUMBER)).Times(1);
  }
  EXPECT_START_LISTENING_FOR_DATA(scanner_controller_).Times(1);

  auto start_future = scanner_controller_.start();
  sendStartReply();
  EXPECT_TRUE(isFutureReady(start_future));
}

TEST_F(ScannerControllerTest, retryAfterStartReplyTimeout)
{
  const unsigned int number_of_retries{ 5 };

  EXPECT_REQUEST_SEND(scanner_controller_, StartRequest(scanner_config_, DEFAULT_START_REQUEST_SEQ_NUMBER))
      .Times(number_of_retries);

  auto start_future = scanner_controller_.start();
  for (unsigned int i = 0; i < (number_of_retries - 1); ++i)
  {
    simulateUdpTimeout("Udp timeout");
  }
  sendStartReply();
  EXPECT_TRUE(isFutureReady(start_future));
}

TEST_F(ScannerControllerTest, receivingMultipleStartReplies)
{
  const UDPFrameTestDataWithoutIntensities test_data;
  const LaserScan scan{ testDataToLaserScan(test_data) };

  EXPECT_CALL(mock_, laserscan_callback(scan)).Times(2);

  scanner_controller_.start();
  simulateUdpTimeout("Udp timeout");
  sendStartReply();
  sendMonitoringFrame(test_data);
  sendStartReply();
  sendMonitoringFrame(test_data);
}

TEST_F(ScannerControllerTest, successfulStopSequence)
{
  {
    InSequence seq;
    EXPECT_START_LISTENING_FOR_CONTROL(scanner_controller_).Times(1);
    EXPECT_REQUEST_SEND(scanner_controller_, StopRequest()).Times(1);
  }
  auto stop_future = scanner_controller_.stop();
  sendStopReply();
  EXPECT_TRUE(isFutureReady(stop_future));
}

TEST_F(ScannerControllerTest, stopWhileWaitingForStartReply)
{
  {
    InSequence seq;
    EXPECT_REQUEST_SEND(scanner_controller_, StartRequest(scanner_config_, DEFAULT_START_REQUEST_SEQ_NUMBER)).Times(1);
    EXPECT_REQUEST_SEND(scanner_controller_, StopRequest()).Times(1);
  }

  scanner_controller_.sendStartRequest();
  auto stop_future = scanner_controller_.stop();
  sendStopReply();
  EXPECT_TRUE(isFutureReady(stop_future));
}

TEST_F(ScannerControllerTest, stopReplyTimeout)
{
  // Has no defined behaviour yet
  {
    InSequence seq;
    EXPECT_CALL(scanner_controller_.control_udp_client_, startAsyncReceiving(_, _, _)).Times(1);
    EXPECT_CALL(scanner_controller_.control_udp_client_, write(StopRequest().serialize())).Times(1);
  }

  scanner_controller_.stop();
  simulateUdpTimeout("Udp timeout");
  sendStopReply();
}

TEST_F(ScannerControllerTest, handleMonitoringFrame)
{
  const UDPFrameTestDataWithoutIntensities test_data;
  const LaserScan scan{ testDataToLaserScan(test_data) };

  EXPECT_CALL(mock_, laserscan_callback(scan)).Times(1);

  scanner_controller_.start();
  sendStartReply();
  sendMonitoringFrame(test_data);
}

TEST_F(ScannerControllerTest, handleEmptyMonitoringFrame)
{
  EXPECT_CALL(mock_, laserscan_callback(_)).Times(0);

  scanner_controller_.start();
  sendStartReply();

  const UDPFrameTestDataWithoutMeasurementsAndIntensities test_data;
  sendMonitoringFrame(test_data);
}

TEST_F(ScannerControllerTest, handleEarlyMonitoringFrame)
{
  EXPECT_CALL(mock_, laserscan_callback(_)).Times(0);

  scanner_controller_.start();

  const UDPFrameTestDataWithoutIntensities test_data;
  sendMonitoringFrame(test_data);
}

TEST_F(ScannerControllerTest, handleLateMonitoringFrame)
{
  EXPECT_CALL(mock_, laserscan_callback(_)).Times(0);

  scanner_controller_.start();
  sendStartReply();

  scanner_controller_.stop();

  const UDPFrameTestDataWithoutIntensities test_data;
  sendMonitoringFrame(test_data);

  sendStopReply();
}

TEST_F(ScannerControllerTest, handleError)
{
  simulateUdpError("Udp error");  // only for coverage for now
}

TEST_F(ScannerControllerTest, constructorInvalidLaserScanCallback)
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
