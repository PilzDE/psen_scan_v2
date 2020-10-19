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
#include <functional>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <pilz_testutils/mock_appender.h>
#include <pilz_testutils/logger_mock.h>

#include <rosconsole_bridge/bridge.h>
REGISTER_ROSCONSOLE_BRIDGE;

// Test frameworks
#include "psen_scan_v2/async_barrier.h"
#include "psen_scan_v2/mock_udp_server.h"
#include "psen_scan_v2/udp_frame_dumps.h"
#include "psen_scan_v2/raw_data_array_conversion.h"

// Software under testing
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/scanner_reply_msg.h"
#include "psen_scan_v2/scan_range.h"
#include "psen_scan_v2/diagnostics.h"

namespace psen_scan_v2_test
{
using namespace psen_scan_v2;

static const std::string SCANNER_IP_ADDRESS{ "127.0.0.1" };
static constexpr unsigned short CONTROL_PORT_OF_SCANNER_DEVICE{ 3000 };
static constexpr unsigned short DATA_PORT_OF_SCANNER_DEVICE{ 2000 };

static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };
static constexpr uint32_t HOST_UDP_PORT_DATA{ 45000 };
static constexpr uint32_t HOST_UDP_PORT_CONTROL{ 57000 };

static constexpr DefaultScanRange SCAN_RANGE{ TenthOfDegree(0), TenthOfDegree(1) };

static constexpr std::chrono::milliseconds WAIT_TIMEOUT{ 10 };
static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 3 };

static constexpr uint32_t DEFAULT_SEQ_NUMBER{ 0u };
static constexpr bool DIAGNOSTICS_ENABLED{ false };

using std::placeholders::_1;
using std::placeholders::_2;

using namespace ::testing;
using namespace std::chrono_literals;
using namespace pilz_testutils;

ACTION_P(OpenBarrier, barrier)
{
  barrier->release();
}

class UserCallbacks
{
public:
  MOCK_METHOD1(LaserScanCallback, void(const LaserScan&));
};

class ScannerMock
{
public:
  MOCK_METHOD2(receiveControlMsg, void(const udp::endpoint&, const psen_scan_v2::DynamicSizeRawData&));
  MOCK_METHOD2(receiveDataMsg, void(const udp::endpoint&, const psen_scan_v2::DynamicSizeRawData&));

public:
  void startListeningForControlMsg();
  void startContinuousListeningForControlMsg();

public:
  void sendStartReply();
  void sendStopReply();
  void sendMonitoringFrame(MonitoringFrameMsg& msg);

private:
  void sendReply(const uint32_t reply_type);

private:
  const udp::endpoint control_msg_receiver_{ udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS),
                                                           HOST_UDP_PORT_CONTROL) };

  const udp::endpoint monitoring_frame_receiver_{
    udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS), HOST_UDP_PORT_DATA)
  };

  MockUDPServer control_server_{ CONTROL_PORT_OF_SCANNER_DEVICE,
                                 std::bind(&ScannerMock::receiveControlMsg, this, _1, _2) };
  MockUDPServer data_server_{ DATA_PORT_OF_SCANNER_DEVICE, std::bind(&ScannerMock::receiveDataMsg, this, _1, _2) };
};

void ScannerMock::startListeningForControlMsg()
{
  control_server_.asyncReceive();
}

void ScannerMock::startContinuousListeningForControlMsg()
{
  control_server_.asyncReceive(MockUDPServer::ReceiveMode::continuous);
}

void ScannerMock::sendReply(const uint32_t reply_type)
{
  const ScannerReplyMsg msg(reply_type, 0x00);
  control_server_.asyncSend<REPLY_MSG_FROM_SCANNER_SIZE>(control_msg_receiver_, msg.serialize());
}

void ScannerMock::sendStartReply()
{
  sendReply(getOpCodeValue(ScannerReplyMsgType::Start));
}

void ScannerMock::sendStopReply()
{
  sendReply(getOpCodeValue(ScannerReplyMsgType::Stop));
}

void ScannerMock::sendMonitoringFrame(MonitoringFrameMsg& msg)
{
  DynamicSizeRawData dynamic_raw_scan = serialize(msg);
  MaxSizeRawData max_size_raw_data = convertToMaxSizeRawData(dynamic_raw_scan);

  data_server_.asyncSend<max_size_raw_data.size()>(monitoring_frame_receiver_, max_size_raw_data);
}

ScannerConfiguration createScannerConfig()
{
  return ScannerConfiguration(
      HOST_IP_ADDRESS, HOST_UDP_PORT_DATA, HOST_UDP_PORT_CONTROL, SCANNER_IP_ADDRESS, SCAN_RANGE, DIAGNOSTICS_ENABLED);
}

TEST(ScannerAPITests, testStartFunctionality)
{
  StrictMock<ScannerMock> scanner_mock;
  const ScannerConfiguration config{ createScannerConfig() };
  UserCallbacks cb;
  Scanner scanner(config, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));
  const StartRequest start_req(config, DEFAULT_SEQ_NUMBER);

  Barrier start_req_received_barrier;
  EXPECT_CALL(scanner_mock, receiveControlMsg(_, start_req.serialize()))
      .WillOnce(OpenBarrier(&start_req_received_barrier));

  scanner_mock.startListeningForControlMsg();
  const auto start_future{ std::async(std::launch::async, [&scanner]() {
    const auto start_future = scanner.start();
    start_future.wait();
  }) };

  EXPECT_TRUE(start_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Start request not received";
  EXPECT_EQ(start_future.wait_for(WAIT_TIMEOUT), std::future_status::timeout) << "Scanner::start() finished too early";
  scanner_mock.sendStartReply();
  EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
}

TEST(ScannerAPITests, testStopFunctionality)
{
  StrictMock<ScannerMock> scanner_mock;
  const ScannerConfiguration config{ createScannerConfig() };
  UserCallbacks cb;
  Scanner scanner(config, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  EXPECT_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config, DEFAULT_SEQ_NUMBER).serialize()))
      .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  Barrier stop_req_received_barrier;
  EXPECT_CALL(scanner_mock, receiveControlMsg(_, StopRequest().serialize()))
      .WillOnce(OpenBarrier(&stop_req_received_barrier));

  scanner_mock.startListeningForControlMsg();
  const auto scanner_start = scanner.start();
  scanner_start.wait();

  scanner_mock.startListeningForControlMsg();
  const auto stop_future{ std::async(std::launch::async, [&scanner]() {
    const auto stop_future = scanner.stop();
    stop_future.wait();
  }) };

  EXPECT_TRUE(stop_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Stop request not received";
  EXPECT_EQ(stop_future.wait_for(WAIT_TIMEOUT), std::future_status::timeout) << "Scanner::stop() finished too early";
  scanner_mock.sendStopReply();
  EXPECT_EQ(stop_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::stop() not finished";
}

TEST(ScannerAPITests, testStartReplyTimeout)
{
  StrictMock<ScannerMock> scanner_mock;
  const ScannerConfiguration config{ createScannerConfig() };
  UserCallbacks cb;
  Scanner scanner(config, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  EXPECT_CALL(scanner_mock, receiveControlMsg(_, _)).Times(AtLeast(2));

  scanner_mock.startContinuousListeningForControlMsg();
  const auto start_future{ std::async(std::launch::async, [&scanner]() {
    const auto start_future = scanner.start();
    start_future.wait();
  }) };

  EXPECT_EQ(start_future.wait_for(3000ms), std::future_status::timeout) << "Scanner::start() finished too early";
  scanner_mock.sendStartReply();
  EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
}

TEST(ScannerAPITests, testReceivingOfMonitoringFrame)
{
  pilz_testutils::LoggerMock ros_log_mock;
  StrictMock<ScannerMock> scanner_mock;
  const ScannerConfiguration config{ createScannerConfig() };
  UserCallbacks cb;
  Scanner scanner(config, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  MonitoringFrameMsg msg(
      TenthOfDegree(0), TenthOfDegree(1), 0, { 1, 2, 3, 4, 5 }, { { ScannerId::MASTER, ErrorLocation(1, 7) } });

  Barrier monitoring_frame_barrier;
  Barrier diagnostic_barrier;
  {
    InSequence seq;

    EXPECT_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config, DEFAULT_SEQ_NUMBER).serialize()))
        .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

    // Check that toLaserScan(msg) == arg
    EXPECT_CALL(cb, LaserScanCallback(toLaserScan(msg))).WillOnce(OpenBarrier(&monitoring_frame_barrier));
  }

  EXPECT_LOG(*ros_log_mock, INFO, "Start scanner called.").Times(1);
  EXPECT_LOG(*ros_log_mock, INFO, "Scanner started successfully.").Times(1);
  EXPECT_LOG(
      *ros_log_mock, WARN, "{Device: Master - Alarm: The front panel of the safety laser scanner must be cleaned.}")
      .Times(1)
      .WillOnce(OpenBarrier(&diagnostic_barrier));

  scanner_mock.startListeningForControlMsg();
  auto promis = scanner.start();
  promis.wait_for(DEFAULT_TIMEOUT);

  scanner_mock.sendMonitoringFrame(msg);

  EXPECT_TRUE(monitoring_frame_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Monitoring frame not received";
  EXPECT_TRUE(diagnostic_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Diagnostic message not received";
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
