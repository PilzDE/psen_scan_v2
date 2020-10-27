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
#include "psen_scan_v2/logging.h"

// Software under testing
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scanner_v2.h"
#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/scanner_reply_msg.h"
#include "psen_scan_v2/scan_range.h"
#include "psen_scan_v2/diagnostics.h"

namespace psen_scan_v2_test
{
using namespace psen_scan_v2;

static const std::string SCANNER_IP_ADDRESS{ "127.0.0.1" };
static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };

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

static MonitoringFrameMsg createValidMonitoringFrameMsg()
{
  const auto from_theta{ TenthOfDegree(10.) };
  const auto resolution{ TenthOfDegree(3.14 / 2.) };
  const uint32_t scan_counter{ 42 };
  const std::vector<double> measurements{ 1., 2., 3., 4.5, 5., 42. };
  const std::vector<double> intensities{ 0., 4., 3., 1007., 508., 14000. };
  const std::vector<MonitoringFrameDiagnosticMessage> diagnostic_messages{ { ScannerId::MASTER, ErrorLocation(1, 7) } };

  return MonitoringFrameMsg(from_theta, resolution, scan_counter, measurements, intensities, diagnostic_messages);
}

struct PortHolder
{
  PortHolder& operator++()
  {
    data_port_host = (data_port_host + 1) % MAX_DATA_PORT_HOST;
    if (data_port_host == 0)
    {
      data_port_host = MIN_DATA_PORT_HOST;
    }

    control_port_host = (control_port_host + 1) % MAX_CONTROL_PORT_HOST;
    if (control_port_host == 0)
    {
      control_port_host = MIN_CONTROL_PORT_HOST;
    }

    control_port_scanner = (control_port_scanner + 1) % MAX_CONTROL_PORT_SCANNER;
    if (control_port_scanner == 0)
    {
      control_port_scanner = MIN_CONTROL_PORT_HOST;
    }

    data_port_scanner = (data_port_scanner + 1) % MAX_DATA_PORT_SCANNER;
    if (data_port_scanner == 0)
    {
      data_port_scanner = MIN_CONTROL_PORT_HOST;
    }

    return *this;
  }

  void printPorts() const
  {
    std::cout << "Host ports:\n"
              << "- data port = " << data_port_host << "\n"
              << "- control port = " << control_port_host << "\n"
              << "Scanner ports:\n"
              << "- data port = " << control_port_scanner << "\n"
              << "- control port = " << data_port_scanner << "\n"
              << std::endl;
  }

  const int MIN_DATA_PORT_HOST{ 45000 };
  const int MAX_DATA_PORT_HOST{ 46000 };

  const int MIN_CONTROL_PORT_HOST{ 57000 };
  const int MAX_CONTROL_PORT_HOST{ 58000 };

  const unsigned short MIN_CONTROL_PORT_SCANNER{ 3000u };
  const unsigned short MAX_CONTROL_PORT_SCANNER{ 4000u };

  const unsigned short MIN_DATA_PORT_SCANNER{ 7000u };
  const unsigned short MAX_DATA_PORT_SCANNER{ 8000u };

  int data_port_host{ MIN_DATA_PORT_HOST };
  int control_port_host{ MIN_CONTROL_PORT_HOST };

  unsigned short control_port_scanner{ MIN_CONTROL_PORT_SCANNER };
  unsigned short data_port_scanner{ MIN_DATA_PORT_SCANNER };
};

static PortHolder GLOBAL_PORT_HOLDER;

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
  ScannerMock(const PortHolder& port_holder)
    : control_msg_receiver_(
          udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS), port_holder.control_port_host))
    , monitoring_frame_receiver_(
          udp::endpoint(boost::asio::ip::address_v4::from_string(HOST_IP_ADDRESS), port_holder.data_port_host))
    , control_server_(port_holder.control_port_scanner, std::bind(&ScannerMock::receiveControlMsg, this, _1, _2))
    , data_server_(port_holder.data_port_scanner, std::bind(&ScannerMock::receiveDataMsg, this, _1, _2))
  {
  }

public:
  MOCK_METHOD2(receiveControlMsg, void(const udp::endpoint&, const psen_scan_v2::DynamicSizeRawData&));
  MOCK_METHOD2(receiveDataMsg, void(const udp::endpoint&, const psen_scan_v2::DynamicSizeRawData&));

public:
  void startListeningForControlMsg();
  void startContinuousListeningForControlMsg();

public:
  void sendStartReply();
  void sendStopReply();
  void sendMonitoringFrame(const MonitoringFrameMsg& msg);
  void sendEmptyMonitoringFrame();

private:
  void sendReply(const uint32_t reply_type);

private:
  const udp::endpoint control_msg_receiver_;
  const udp::endpoint monitoring_frame_receiver_;

  MockUDPServer control_server_;
  MockUDPServer data_server_;
};

class ScannerAPITests : public testing::Test
{
protected:
  void SetUp() override;

protected:
  const PortHolder port_holder_{ ++GLOBAL_PORT_HOLDER };

  ScannerConfiguration config_{
    HOST_IP_ADDRESS, port_holder_.data_port_host, port_holder_.control_port_host, SCANNER_IP_ADDRESS,
    SCAN_RANGE,      DIAGNOSTICS_ENABLED
  };
};

void ScannerAPITests::SetUp()
{
  port_holder_.printPorts();
}

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
  std::cout << "ScannerMock: Send start reply..." << std::endl;
  sendReply(getOpCodeValue(ScannerReplyMsgType::Start));
}

void ScannerMock::sendStopReply()
{
  sendReply(getOpCodeValue(ScannerReplyMsgType::Stop));
}

void ScannerMock::sendMonitoringFrame(const MonitoringFrameMsg& msg)
{
  std::cout << "ScannerMock: Send monitoring frame..." << std::endl;
  DynamicSizeRawData dynamic_raw_scan = serialize(msg);
  MaxSizeRawData max_size_raw_data = convertToMaxSizeRawData(dynamic_raw_scan);

  data_server_.asyncSend<max_size_raw_data.size()>(monitoring_frame_receiver_, max_size_raw_data);
}

void ScannerMock::sendEmptyMonitoringFrame()
{
  const psen_scan_v2::FixedSizeRawData<0> data;
  data_server_.asyncSend<data.size()>(monitoring_frame_receiver_, data);
}

TEST_F(ScannerAPITests, testStartFunctionality)
{
  StrictMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  ScannerV2 scanner(config_,
                    std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1),
                    port_holder_.data_port_scanner,
                    port_holder_.control_port_scanner);
  const StartRequest start_req(config_, DEFAULT_SEQ_NUMBER);

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

// TEST_F(ScannerAPITests, shouldThrowWhenStartIsCalledTwice)
//{
//  NiceMock<ScannerMock> scanner_mock{ port_holder_ };
//  UserCallbacks cb;
//  ScannerV2 scanner(config_,
//                    std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1),
//                    port_holder_.data_port_scanner,
//                    port_holder_.control_port_scanner);

//  Barrier start_req_received_barrier;
//  ON_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config_, DEFAULT_SEQ_NUMBER).serialize()))
//      .WillByDefault(OpenBarrier(&start_req_received_barrier));

//  scanner_mock.startListeningForControlMsg();
//  const auto start_future = scanner.start();
//  EXPECT_THROW(scanner.start();, std::runtime_error);

//  // EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
//}

TEST_F(ScannerAPITests, startShouldSucceedDespiteUnexpectedMonitoringFrame)
{
  NiceMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  ScannerV2 scanner(config_,
                    std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1),
                    port_holder_.data_port_scanner,
                    port_holder_.control_port_scanner);

  Barrier start_req_received_barrier;
  ON_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config_, DEFAULT_SEQ_NUMBER).serialize()))
      .WillByDefault(OpenBarrier(&start_req_received_barrier));
  EXPECT_CALL(cb, LaserScanCallback(_)).Times(0);

  scanner_mock.startListeningForControlMsg();
  const auto start_future = scanner.start();

  ASSERT_TRUE(start_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Start request not received";

  scanner_mock.sendMonitoringFrame(createValidMonitoringFrameMsg());
  ASSERT_EQ(start_future.wait_for(WAIT_TIMEOUT), std::future_status::timeout) << "Scanner::start() finished too early ";

  scanner_mock.sendStartReply();
  EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
}

TEST_F(ScannerAPITests, testStopFunctionality)
{
  StrictMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  ScannerV2 scanner(config_,
                    std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1),
                    port_holder_.data_port_scanner,
                    port_holder_.control_port_scanner);

  EXPECT_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config_, DEFAULT_SEQ_NUMBER).serialize()))
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

TEST_F(ScannerAPITests, shouldThrowWhenStopIsCalledTwice)
{
  StrictMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  ScannerV2 scanner(config_,
                    std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1),
                    port_holder_.data_port_scanner,
                    port_holder_.control_port_scanner);

  EXPECT_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config_, DEFAULT_SEQ_NUMBER).serialize()))
      .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  Barrier stop_req_received_barrier;
  EXPECT_CALL(scanner_mock, receiveControlMsg(_, StopRequest().serialize()))
      .WillOnce(OpenBarrier(&stop_req_received_barrier));

  scanner_mock.startListeningForControlMsg();
  const auto start_future = scanner.start();
  std::cout << "Start() function finished" << std::endl;
  start_future.wait();

  scanner_mock.startListeningForControlMsg();
  auto stop_future = scanner.stop();
  EXPECT_THROW(scanner.stop();, std::runtime_error);

  EXPECT_TRUE(stop_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Stop request not received";
  scanner_mock.sendStopReply();
  EXPECT_EQ(stop_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::stop() not finished";
}

TEST_F(ScannerAPITests, testStartReplyTimeout)
{
  NiceMock<pilz_testutils::LoggerMock> ros_log_mock;
  StrictMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  ScannerV2 scanner(config_,
                    std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1),
                    port_holder_.data_port_scanner,
                    port_holder_.control_port_scanner);

  Barrier error_msg_barrier;
  Barrier twice_called_barrier;
  {
    InSequence seq;
    EXPECT_CALL(scanner_mock, receiveControlMsg(_, _)).Times(1);
    EXPECT_CALL(scanner_mock, receiveControlMsg(_, _)).Times(1).WillOnce(OpenBarrier(&twice_called_barrier));
  }
  EXPECT_LOG(*ros_log_mock, INFO, "Start scanner called.").Times(1);
  EXPECT_LOG(*ros_log_mock,
             ERROR,
             "Timeout while waiting for the scanner to start! Retrying... "
             "(Please check the ethernet connection or contact PILZ support if the error persists.)")
      .Times(AtLeast(1))
      .WillOnce(OpenBarrier(&error_msg_barrier));
  EXPECT_LOG(*ros_log_mock, INFO, "Scanner started successfully.").Times(1);

  scanner_mock.startContinuousListeningForControlMsg();
  const auto start_future{ std::async(std::launch::async, [&scanner]() {
    const auto scanner_start = scanner.start();
    scanner_start.wait();
  }) };

  EXPECT_TRUE(error_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Error message not received";
  EXPECT_TRUE(twice_called_barrier.waitTillRelease(5000ms)) << "Start reply not send at least twice in time";
  scanner_mock.sendStartReply();
  EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
}

TEST_F(ScannerAPITests, LaserScanShouldContainAllInfosTransferedByMonitoringFrameMsg)
{
  pilz_testutils::LoggerMock ros_log_mock;
  StrictMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;

  ScannerV2 scanner(config_,
                    std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1),
                    port_holder_.data_port_scanner,
                    port_holder_.control_port_scanner);

  MonitoringFrameMsg msg{ createValidMonitoringFrameMsg() };
  Barrier monitoring_frame_barrier;
  Barrier diagnostic_barrier;
  {
    InSequence seq;

    EXPECT_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config_, DEFAULT_SEQ_NUMBER).serialize()))
        .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

    // Check that toLaserScan(msg) == arg
    EXPECT_CALL(cb, LaserScanCallback(toLaserScan(msg))).WillOnce(OpenBarrier(&monitoring_frame_barrier));
  }

  EXPECT_LOG(*ros_log_mock, INFO, "Start scanner called.").Times(1);
  EXPECT_LOG(*ros_log_mock, INFO, "Scanner started successfully.").Times(1);
  EXPECT_LOG(*ros_log_mock,
             WARN,
             "The scanner reports an error: {Device: Master - Alarm: The front panel of the safety laser scanner must "
             "be cleaned.}")
      .Times(1)
      .WillOnce(OpenBarrier(&diagnostic_barrier));

  scanner_mock.startListeningForControlMsg();
  auto promis = scanner.start();
  promis.wait_for(DEFAULT_TIMEOUT);

  scanner_mock.sendMonitoringFrame(msg);

  EXPECT_TRUE(monitoring_frame_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Monitoring frame not received";
  EXPECT_TRUE(diagnostic_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Diagnostic message not received";
}

TEST_F(ScannerAPITests, shouldNotCallLaserscanCallbackInCaseOfEmptyMonitoringFrame)
{
  StrictMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  ScannerV2 scanner(config_,
                    std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1),
                    port_holder_.data_port_scanner,
                    port_holder_.control_port_scanner);

  Barrier monitoring_frame_barrier;
  {
    InSequence seq;

    EXPECT_CALL(scanner_mock, receiveControlMsg(_, StartRequest(config_, DEFAULT_SEQ_NUMBER).serialize()))
        .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

    EXPECT_CALL(cb, LaserScanCallback(_)).WillOnce(OpenBarrier(&monitoring_frame_barrier));
  }

  scanner_mock.startListeningForControlMsg();
  auto promis = scanner.start();
  promis.wait_for(DEFAULT_TIMEOUT);

  std::cout << "ScannerAPITests: Send empty monitoring frame..." << std::endl;
  scanner_mock.sendEmptyMonitoringFrame();
  EXPECT_FALSE(monitoring_frame_barrier.waitTillRelease(DEFAULT_TIMEOUT))
      << "Empty monitoring frame triggered LaserScanCallback";

  std::cout << "ScannerAPITests: Send valid monitoring frame..." << std::endl;
  scanner_mock.sendMonitoringFrame(createValidMonitoringFrameMsg());
  EXPECT_TRUE(monitoring_frame_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Monitoring frame not received";
}

TEST_F(ScannerAPITests, shouldThrowWhenConstructedWithInvalidLaserScanCallback)
{
  EXPECT_THROW(ScannerV2 scanner(config_, nullptr);, std::invalid_argument);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
