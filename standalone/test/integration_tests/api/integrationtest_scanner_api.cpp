// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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
#include <random>
#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

// Test frameworks
#include "psen_scan_v2_standalone/mock_console_bridge_output_handler.h"
#include "psen_scan_v2_standalone/communication_layer/mock_udp_server.h"
#include "psen_scan_v2_standalone/communication_layer/udp_frame_dumps.h"
#include "psen_scan_v2_standalone/data_conversion_layer/raw_data_array_conversion.h"

// Software under testing
#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/data_conversion_layer/diagnostics.h"
#include "psen_scan_v2_standalone/logging.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_serialization.h"
#include "psen_scan_v2_standalone/configuration/scanner_configuration.h"
#include "psen_scan_v2_standalone/configuration/scanner_config_builder.h"
#include "psen_scan_v2_standalone/api/scanner_v2.h"
#include "psen_scan_v2_standalone/data_conversion_layer/start_request.h"
#include "psen_scan_v2_standalone/data_conversion_layer/start_request_serialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_serialization_deserialization.h"
#include "psen_scan_v2_standalone/configuration/scan_range.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;

using MaxSizeUdpRawData = std::array<char, 65507>;

static const std::string SCANNER_IP_ADDRESS{ "127.0.0.1" };
static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };

static constexpr configuration::DefaultScanRange SCAN_RANGE{ TenthOfDegree(0), TenthOfDegree(60) };

static constexpr std::chrono::milliseconds WAIT_TIMEOUT{ 10 };
static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 3 };

using std::placeholders::_1;
using std::placeholders::_2;

using namespace ::testing;
using namespace std::chrono_literals;

static double randDouble(double low, double high)
{
  static std::default_random_engine re{};
  using Dist = std::uniform_real_distribution<double>;
  static Dist uid{};
  return uid(re, Dist::param_type{ low, high });
}

static double restrictToOneDigitsAfterComma(const double& value)
{
  return std::round(value * 10.) / 10.;
}

static std::vector<double> generateMeasurements(const unsigned int& num_elements, const double& low, const double& high)
{
  std::vector<double> vec(num_elements);
  // The scanner sends tenth degree values. Therefore, restrict values to one digit after the comma.
  std::generate(vec.begin(), vec.end(), [low, high]() { return restrictToOneDigitsAfterComma(randDouble(low, high)); });
  return vec;
}

static std::vector<double> generateIntensities(const unsigned int& num_elements, const double& low, const double& high)
{
  std::vector<double> vec(num_elements);
  // The scanner sends intensities as int values, therefore, the values are rounded.
  std::generate(vec.begin(), vec.end(), [low, high]() { return std::round(randDouble(low, high)); });
  return vec;
}

static data_conversion_layer::monitoring_frame::Message
createValidMonitoringFrameMsg(const uint32_t scan_counter = 42,
                              const TenthOfDegree start_angle = SCAN_RANGE.getStart(),
                              const TenthOfDegree end_angle = SCAN_RANGE.getEnd())
{
  const auto resolution{ TenthOfDegree(10) };

  const unsigned int num_elements = (end_angle - start_angle) / resolution;
  const double lowest_measurement{ 0. };
  const double highest_measurement{ 10. };
  const std::vector<double> measurements{ generateMeasurements(num_elements, lowest_measurement, highest_measurement) };

  const double lowest_intensity{ 0. };
  const double highest_intensity{ 17000. };
  const std::vector<double> intensities{ generateIntensities(num_elements, lowest_intensity, highest_intensity) };

  const std::vector<data_conversion_layer::monitoring_frame::diagnostic::Message> diagnostic_messages{
    { ScannerId::master, data_conversion_layer::monitoring_frame::diagnostic::ErrorLocation(1, 7) }
  };

  return data_conversion_layer::monitoring_frame::Message(
      start_angle, resolution, scan_counter, measurements, intensities, diagnostic_messages);
}

static std::vector<data_conversion_layer::monitoring_frame::Message>
createValidMonitoringFrameMsgs(const uint32_t scan_counter, const std::size_t num_elements)
{
  std::vector<data_conversion_layer::monitoring_frame::Message> msgs(num_elements);
  std::generate(msgs.begin(), msgs.end(), [scan_counter]() { return createValidMonitoringFrameMsg(scan_counter); });
  return msgs;
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
      control_port_scanner = MIN_CONTROL_PORT_SCANNER;
    }

    data_port_scanner = (data_port_scanner + 1) % MAX_DATA_PORT_SCANNER;
    if (data_port_scanner == 0)
    {
      data_port_scanner = MIN_DATA_PORT_SCANNER;
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
  MOCK_METHOD1(LaserScanCallback, void(const api::LaserScan&));
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
  MOCK_METHOD2(receiveControlMsg, void(const udp::endpoint&, const psen_scan_v2_standalone::RawData&));
  MOCK_METHOD2(receiveDataMsg, void(const udp::endpoint&, const psen_scan_v2_standalone::RawData&));

public:
  void startListeningForControlMsg();
  void startContinuousListeningForControlMsg();

public:
  void sendStartReply();
  void sendStopReply();
  void sendMonitoringFrame(const data_conversion_layer::monitoring_frame::Message& msg);
  void sendEmptyMonitoringFrame();

private:
  void sendReply(const data_conversion_layer::scanner_reply::Message::Type& reply_type);

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

  configuration::ScannerConfiguration config_{ configuration::ScannerConfigurationBuilder()
                                                   .hostIP(HOST_IP_ADDRESS)
                                                   .hostDataPort(port_holder_.data_port_host)
                                                   .hostControlPort(port_holder_.control_port_host)
                                                   .scannerIp(SCANNER_IP_ADDRESS)
                                                   .scannerDataPort(port_holder_.data_port_scanner)
                                                   .scannerControlPort(port_holder_.control_port_scanner)
                                                   .scanRange(SCAN_RANGE)
                                                   .build() };
};

void ScannerAPITests::SetUp()
{
  port_holder_.printPorts();
  setLogLevel(CONSOLE_BRIDGE_LOG_DEBUG);
}

void ScannerMock::startListeningForControlMsg()
{
  control_server_.asyncReceive();
}

void ScannerMock::startContinuousListeningForControlMsg()
{
  control_server_.asyncReceive(MockUDPServer::ReceiveMode::continuous);
}

void ScannerMock::sendReply(const data_conversion_layer::scanner_reply::Message::Type& reply_type)
{
  const data_conversion_layer::scanner_reply::Message msg(
      reply_type, data_conversion_layer::scanner_reply::Message::OperationResult::accepted);
  control_server_.asyncSend(control_msg_receiver_, data_conversion_layer::scanner_reply::serialize(msg));
}

void ScannerMock::sendStartReply()
{
  std::cout << "ScannerMock: Send start reply..." << std::endl;
  sendReply(data_conversion_layer::scanner_reply::Message::Type::start);
}

void ScannerMock::sendStopReply()
{
  std::cout << "ScannerMock: Send stop reply..." << std::endl;
  sendReply(data_conversion_layer::scanner_reply::Message::Type::stop);
}

void ScannerMock::sendMonitoringFrame(const data_conversion_layer::monitoring_frame::Message& msg)
{
  std::cout << "ScannerMock: Send monitoring frame..." << std::endl;
  data_server_.asyncSend(monitoring_frame_receiver_, data_conversion_layer::monitoring_frame::serialize(msg));
}

void ScannerMock::sendEmptyMonitoringFrame()
{
  psen_scan_v2_standalone::RawData data;
  assert(data.empty());
  data_server_.asyncSend(monitoring_frame_receiver_, data);
}

TEST_F(ScannerAPITests, testStartFunctionality)
{
  StrictMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));
  const data_conversion_layer::start_request::Message start_req(config_);

  util::Barrier start_req_received_barrier;
  EXPECT_CALL(scanner_mock, receiveControlMsg(_, data_conversion_layer::start_request::serialize(start_req)))
      .WillOnce(OpenBarrier(&start_req_received_barrier));

  scanner_mock.startListeningForControlMsg();
  const auto start_future{ std::async(std::launch::async, [&scanner]() {
    const auto start_future = scanner.start();
    start_future.wait();
  }) };

  ASSERT_TRUE(start_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Start request not received";
  ASSERT_EQ(start_future.wait_for(WAIT_TIMEOUT), std::future_status::timeout) << "Scanner::start() finished too early";
  scanner_mock.sendStartReply();
  ASSERT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
}

TEST_F(ScannerAPITests, shouldReturnInvalidFutureWhenStartIsCalledSecondTime)
{
  NiceMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  scanner_mock.startListeningForControlMsg();
  const auto start_future = scanner.start();
  EXPECT_TRUE(start_future.valid()) << "First call too Scanner::start() should return VALID std::future";
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_FALSE(scanner.start().valid()) << "Subsequenct calls to Scanner::start() should return INVALID std::future";
  }
  scanner_mock.sendStartReply();
  EXPECT_EQ(start_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::start() not finished";
}

TEST_F(ScannerAPITests, startShouldSucceedDespiteUnexpectedMonitoringFrame)
{
  NiceMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  util::Barrier start_req_received_barrier;
  ON_CALL(
      scanner_mock,
      receiveControlMsg(
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config_))))
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
  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  EXPECT_CALL(
      scanner_mock,
      receiveControlMsg(
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config_))))
      .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  util::Barrier stop_req_received_barrier;
  EXPECT_CALL(scanner_mock, receiveControlMsg(_, data_conversion_layer::stop_request::serialize()))
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

TEST_F(ScannerAPITests, shouldReturnInvalidFutureWhenStopIsCalledSecondTime)
{
  NiceMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  ON_CALL(
      scanner_mock,
      receiveControlMsg(
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config_))))
      .WillByDefault(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  scanner_mock.startListeningForControlMsg();
  const auto start_future = scanner.start();
  start_future.wait();

  scanner_mock.startListeningForControlMsg();
  const auto stop_future = scanner.stop();
  EXPECT_TRUE(stop_future.valid()) << "First call too Scanner::stop() should return VALID std::future";
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_FALSE(scanner.stop().valid()) << "Subsequenct calls to Scanner::stop() should return INVALID std::future";
  }

  scanner_mock.sendStopReply();
  EXPECT_EQ(stop_future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << "Scanner::stop() not finished";
}

TEST_F(ScannerAPITests, testStartReplyTimeout)
{
  INJECT_NICE_LOG_MOCK;
  StrictMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  util::Barrier error_msg_barrier;
  util::Barrier twice_called_barrier;
  {
    InSequence seq;
    EXPECT_CALL(scanner_mock, receiveControlMsg(_, _)).Times(1);
    EXPECT_CALL(scanner_mock, receiveControlMsg(_, _)).Times(1).WillOnce(OpenBarrier(&twice_called_barrier));
  }

  EXPECT_LOG_SHORT(DEBUG, _).Times(AnyNumber());
  EXPECT_LOG_SHORT(INFO, "Scanner: Start scanner called.").Times(1);
  EXPECT_LOG_SHORT(ERROR,
                   "StateMachine: Timeout while waiting for the scanner to start! Retrying... "
                   "(Please check the ethernet connection or contact PILZ support if the error persists.)")
      .Times(AtLeast(1))
      .WillOnce(OpenBarrier(&error_msg_barrier));
  EXPECT_LOG_SHORT(INFO, "ScannerController: Scanner started successfully.").Times(1);

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
  INJECT_LOG_MOCK;
  StrictMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;

  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  const data_conversion_layer::monitoring_frame::Message msg{ createValidMonitoringFrameMsg() };

  util::Barrier monitoring_frame_barrier;
  util::Barrier diagnostic_barrier;
  {
    InSequence seq;
    EXPECT_CALL(
        scanner_mock,
        receiveControlMsg(
            _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config_))))
        .WillOnce(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

    // Check that toLaserScan(msg) == arg
    EXPECT_CALL(cb, LaserScanCallback(data_conversion_layer::toLaserScan(msg)))
        .WillOnce(OpenBarrier(&monitoring_frame_barrier));
  }

  EXPECT_LOG_SHORT(DEBUG, _).Times(AnyNumber());
  EXPECT_LOG_SHORT(INFO, "Scanner: Start scanner called.").Times(1);
  EXPECT_LOG_SHORT(INFO, "ScannerController: Scanner started successfully.").Times(1);
  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: The scanner reports an error: {Device: Master - Alarm: The front panel of the safety "
                   "laser scanner must be cleaned.}")
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
  INJECT_NICE_LOG_MOCK;
  NiceMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  ON_CALL(
      scanner_mock,
      receiveControlMsg(
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config_))))
      .WillByDefault(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  util::Barrier valid_msg_barrier;
  EXPECT_CALL(cb, LaserScanCallback(_)).WillOnce(OpenBarrier(&valid_msg_barrier));

  util::Barrier empty_msg_received;
  // Needed to allow all other log messages which might be received
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_LOG_SHORT(
      WARN,
      "StateMachine: No transition in state \"WaitForMonitoringFrame\" for event \"MonitoringFrameReceivedError\".")
      .Times(1)
      .WillOnce(OpenBarrier(&empty_msg_received));

  scanner_mock.startListeningForControlMsg();
  auto promis = scanner.start();
  promis.wait_for(DEFAULT_TIMEOUT);

  std::cout << "ScannerAPITests: Send empty monitoring frame..." << std::endl;
  scanner_mock.sendEmptyMonitoringFrame();
  EXPECT_TRUE(empty_msg_received.waitTillRelease(DEFAULT_TIMEOUT)) << "Empty monitoring frame not received";

  std::cout << "ScannerAPITests: Send valid monitoring frame..." << std::endl;
  scanner_mock.sendMonitoringFrame(createValidMonitoringFrameMsg());
  EXPECT_TRUE(valid_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Valid monitoring frame not received";
}

TEST_F(ScannerAPITests, shouldThrowWhenConstructedWithInvalidLaserScanCallback)
{
  EXPECT_THROW(api::ScannerV2 scanner(config_, nullptr);, std::invalid_argument);
}

TEST_F(ScannerAPITests, shouldShowUserMsgIfMonitoringFramesAreMissing)
{
  INJECT_LOG_MOCK;
  NiceMock<ScannerMock> scanner_mock{ port_holder_ };
  NiceMock<UserCallbacks> cb;

  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  const std::size_t num_scans_per_round{ 6 };

  // Create valid scan round
  const uint32_t scan_counter_valid_round{ 42 };
  std::vector<data_conversion_layer::monitoring_frame::Message> valid_scan_round_msgs{ createValidMonitoringFrameMsgs(
      scan_counter_valid_round, num_scans_per_round) };

  // Create invalid scan round -> invalid because one MonitoringFrame missing
  const uint32_t scan_counter_invalid_round{ scan_counter_valid_round + 1 };
  std::vector<data_conversion_layer::monitoring_frame::Message> invalid_scan_round_msgs{ createValidMonitoringFrameMsgs(
      scan_counter_invalid_round, num_scans_per_round - 1) };
  invalid_scan_round_msgs.emplace_back(createValidMonitoringFrameMsg(scan_counter_invalid_round + 1));

  ON_CALL(
      scanner_mock,
      receiveControlMsg(
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config_))))
      .WillByDefault(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  util::Barrier user_msg_barrier;
  // Needed to allow all other log messages which might be received
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: Detected dropped MonitoringFrame."
                   " (Please check the ethernet connection or contact PILZ support if the error persists.)")
      .Times(1)
      .WillOnce(OpenBarrier(&user_msg_barrier));

  scanner_mock.startListeningForControlMsg();
  auto start_done = scanner.start();
  start_done.wait_for(DEFAULT_TIMEOUT);

  // Send MonitoringFrames of valid scan round
  for (const auto& msg : valid_scan_round_msgs)
  {
    scanner_mock.sendMonitoringFrame(msg);
    // Sleep to ensure that message are not sent too fast which might cause messages overwrite in socket buffer
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Send MonitoringFrames of invalid scan round
  for (const auto& msg : invalid_scan_round_msgs)
  {
    scanner_mock.sendMonitoringFrame(msg);
    // Sleep to ensure that message are not sent too fast which might cause messages overwrite in socket buffer
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(user_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "User message not received";
}

TEST_F(ScannerAPITests, shouldShowUserMsgIfTooManyMonitoringFramesAreReceived)
{
  INJECT_LOG_MOCK;
  NiceMock<ScannerMock> scanner_mock{ port_holder_ };
  NiceMock<UserCallbacks> cb;

  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  const std::size_t num_scans_per_round{ 6 };

  const uint32_t scan_counter{ 42 };
  std::vector<data_conversion_layer::monitoring_frame::Message> msgs{ createValidMonitoringFrameMsgs(
      scan_counter, num_scans_per_round + 1) };
  msgs.emplace_back(createValidMonitoringFrameMsg(scan_counter + 1));

  ON_CALL(
      scanner_mock,
      receiveControlMsg(
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config_))))
      .WillByDefault(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  util::Barrier user_msg_barrier;
  // Needed to allow all other log messages which might be received
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_LOG_SHORT(WARN, "StateMachine: Unexpected: Too many MonitoringFrames for one scan round received.")
      .Times(1)
      .WillOnce(OpenBarrier(&user_msg_barrier));

  scanner_mock.startListeningForControlMsg();
  auto start_done = scanner.start();
  start_done.wait_for(DEFAULT_TIMEOUT);

  for (const auto& msg : msgs)
  {
    scanner_mock.sendMonitoringFrame(msg);
    // Sleep to ensure that message are not sent too fast which might cause messages overwrite in socket buffer
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(user_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "User message not received";
}

TEST_F(ScannerAPITests, shouldShowUserMsgIfMonitoringFrameReceiveTimeout)
{
  INJECT_LOG_MOCK;
  NiceMock<ScannerMock> scanner_mock{ port_holder_ };
  UserCallbacks cb;
  api::ScannerV2 scanner(config_, std::bind(&UserCallbacks::LaserScanCallback, &cb, std::placeholders::_1));

  ON_CALL(
      scanner_mock,
      receiveControlMsg(
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config_))))
      .WillByDefault(InvokeWithoutArgs([&scanner_mock]() { scanner_mock.sendStartReply(); }));

  util::Barrier user_msg_barrier;
  // Needed to allow all other log messages which might be received
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: Timeout while waiting for MonitoringFrame message."
                   " (Please check the ethernet connection or contact PILZ support if the error persists.)")
      .Times(1)
      .WillOnce(OpenBarrier(&user_msg_barrier));

  scanner_mock.startListeningForControlMsg();
  auto start_done = scanner.start();
  start_done.wait_for(DEFAULT_TIMEOUT);

  EXPECT_TRUE(user_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "User message not received";
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
