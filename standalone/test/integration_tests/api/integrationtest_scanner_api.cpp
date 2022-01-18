// Copyright (c) 2020-2022 Pilz GmbH & Co. KG
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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// Test frameworks
#include "psen_scan_v2_standalone/communication_layer/scanner_mock.h"
#include "psen_scan_v2_standalone/util/integrationtest_helper.h"
#include "psen_scan_v2_standalone/util/gtest_expectations.h"
#include "psen_scan_v2_standalone/util/matchers_and_actions.h"
#include "psen_scan_v2_standalone/util/mock_console_bridge_output_handler.h"

// Software under testing
#include "psen_scan_v2_standalone/data_conversion_layer/start_request.h"
#include "psen_scan_v2_standalone/data_conversion_layer/start_request_serialization.h"
#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/scanner_config_builder.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scanner_v2.h"
#include "psen_scan_v2_standalone/util/async_barrier.h"
#include "psen_scan_v2_standalone/util/timestamp.h"

namespace psen_scan_v2_standalone_test
{
using namespace psen_scan_v2_standalone;

static const bool FRAGMENTED_SCAN{ true };
static const bool UNFRAGMENTED_SCAN{ false };
static const std::string HOST_IP_ADDRESS{ "127.0.0.1" };
static const std::string SCANNER_IP_ADDRESS{ "127.0.0.1" };

static constexpr std::chrono::milliseconds FUTURE_WAIT_TIMEOUT{ 10 };

using namespace ::testing;
using namespace std::chrono_literals;

class UserCallbacks
{
public:
  MOCK_METHOD1(LaserScanCallback, void(const LaserScan&));
};

#define EXPECT_STOP_REQUEST_CALL(hw_mock)                                                                              \
  EXPECT_CALL(hw_mock, receiveControlMsg(_, data_conversion_layer::stop_request::serialize()))

#define EXPECT_START_REQUEST_CALL(hw_mock, config)                                                                     \
  EXPECT_CALL(                                                                                                         \
      hw_mock,                                                                                                         \
      receiveControlMsg(                                                                                               \
          _, data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config))))

#define EXPECT_CALLBACK_WILL_OPEN_BARRIER(cb, msgs, barrier)                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    const auto timestamp{ util::getCurrentTime() };                                                                    \
    const auto scan{ createReferenceScan(msgs, timestamp) };                                                           \
    EXPECT_CALL(cb, LaserScanCallback(AllOf(ScanDataEqual(scan), ScanTimestampsInExpectedTimeframe(scan, timestamp)))) \
        .WillOnce(OpenBarrier(&barrier));                                                                              \
  } while (false)

#define EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock, driver, config)                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    util::Barrier start_req_barrier;                                                                                   \
    std::future<void> start_future;                                                                                    \
    EXPECT_START_REQUEST_CALL(*hw_mock, *config).WillOnce(OpenBarrier(&start_req_barrier));                            \
    EXPECT_NO_BLOCK_NO_THROW(start_future = driver->start(););                                                         \
    EXPECT_TRUE(start_future.valid());                                                                                 \
    start_req_barrier.waitTillRelease(2s);                                                                             \
    hw_mock->sendStartReply();                                                                                         \
    EXPECT_FUTURE_IS_READY(start_future, 2s) << "Scanner::start() not finished";                                       \
  } while (false)

#define EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock, driver)                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    util::Barrier stop_req_barrier;                                                                                    \
    std::future<void> stop_future;                                                                                     \
    EXPECT_STOP_REQUEST_CALL(*hw_mock).WillOnce(OpenBarrier(&stop_req_barrier));                                       \
    EXPECT_NO_BLOCK_NO_THROW(stop_future = driver->stop(););                                                           \
    EXPECT_TRUE(stop_future.valid());                                                                                  \
    stop_req_barrier.waitTillRelease(2s);                                                                              \
    hw_mock->sendStopReply();                                                                                          \
    EXPECT_FUTURE_IS_READY(stop_future, 2s) << "Scanner::stop() not finished";                                         \
  } while (false)

class ScannerAPITests : public testing::Test
{
protected:
  void SetUp() override;
  void setUpScannerConfig(const std::string& host_ip = HOST_IP_ADDRESS, bool fragmented = FRAGMENTED_SCAN);
  void setUpScannerV2Driver();
  void setUpScannerHwMock();
  ScannerConfiguration generateScannerConfig(const std::string& host_ip, bool fragmented);

protected:
  const PortHolder port_holder_{ ++GLOBAL_PORT_HOLDER };
  std::unique_ptr<ScannerConfiguration> config_;
  UserCallbacks user_callbacks_;
  std::unique_ptr<ScannerV2> driver_;
  std::unique_ptr<StrictMock<ScannerMock>> hw_mock_;
};

void ScannerAPITests::SetUp()
{
  port_holder_.printPorts();
  setLogLevel(CONSOLE_BRIDGE_LOG_DEBUG);
}

void ScannerAPITests::setUpScannerConfig(const std::string& host_ip, bool fragmented)
{
  config_.reset(new ScannerConfiguration(generateScannerConfig(host_ip, fragmented)));
}

ScannerConfiguration ScannerAPITests::generateScannerConfig(const std::string& host_ip, bool fragmented)
{
  return ScannerConfigurationBuilder(SCANNER_IP_ADDRESS)
      .hostIP(host_ip)
      .hostDataPort(port_holder_.data_port_host)
      .hostControlPort(port_holder_.control_port_host)
      .scannerDataPort(port_holder_.data_port_scanner)
      .scannerControlPort(port_holder_.control_port_scanner)
      .scanRange(DEFAULT_SCAN_RANGE)
      .scanResolution(DEFAULT_SCAN_RESOLUTION)
      .enableIntensities()
      .enableFragmentedScans(fragmented);
}

void ScannerAPITests::setUpScannerV2Driver()
{
  driver_.reset(
      new ScannerV2(*config_, std::bind(&UserCallbacks::LaserScanCallback, &user_callbacks_, std::placeholders::_1)));
}

void ScannerAPITests::setUpScannerHwMock()
{
  hw_mock_.reset(new StrictMock<ScannerMock>{ HOST_IP_ADDRESS, port_holder_ });
}

class ScannerAPITestsFragmented : public ScannerAPITests
{
  void SetUp() override
  {
    ScannerAPITests::SetUp();
    setUpScannerConfig();
    setUpScannerV2Driver();
    setUpScannerHwMock();
  }
};

class ScannerAPITestsUnfragmented : public ScannerAPITests
{
  void SetUp() override
  {
    ScannerAPITests::SetUp();
    setUpScannerConfig(HOST_IP_ADDRESS, UNFRAGMENTED_SCAN);
    setUpScannerV2Driver();
    setUpScannerHwMock();
  }
};

typedef ScannerAPITestsUnfragmented ScannerAPITestsDefaultSetUp;

TEST_F(ScannerAPITests, shouldReceiveStartRequestWithCorrectHostIpWhenUsingAutoInConfigAsHostIp)
{
  setUpScannerConfig("auto");
  setUpScannerV2Driver();
  setUpScannerHwMock();
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, generateScannerConfig(HOST_IP_ADDRESS, FRAGMENTED_SCAN))
      .WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future;
  EXPECT_NO_BLOCK_NO_THROW(start_future = driver_->start(););

  start_req_received_barrier.waitTillRelease(2s);
  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, 2s) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITests, shouldThrowWhenConstructedWithInvalidLaserScanCallback)
{
  setUpScannerConfig();
  EXPECT_THROW(ScannerV2 scanner(*config_, nullptr);, std::invalid_argument);
}

TEST_F(ScannerAPITestsDefaultSetUp, shouldSendStartRequestAndReturnValidFutureWhenLaunchingWithValidConfig)
{
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future;
  EXPECT_NO_BLOCK_NO_THROW(start_future = driver_->start(););

  start_req_received_barrier.waitTillRelease(2s);
  EXPECT_FUTURE_TIMEOUT(start_future, FUTURE_WAIT_TIMEOUT) << "Scanner::start() finished without receiveing reply";
  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, 2s) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITestsDefaultSetUp, shouldReturnInvalidFutureWhenStartIsCalledSecondTime)
{
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future;
  EXPECT_NO_BLOCK_NO_THROW(start_future = driver_->start(););
  EXPECT_TRUE(start_future.valid()) << "First call too Scanner::start() should return VALID std::future";
  for (int i = 0; i < 5; ++i)
  {
    std::future<void> second_start_future;
    EXPECT_NO_BLOCK_NO_THROW(second_start_future = driver_->start(););
    EXPECT_FALSE(second_start_future.valid()) << "Subsequenct calls to Scanner::start() should return INVALID "
                                                 "std::future";
  }
  start_req_received_barrier.waitTillRelease(2s);
  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, 2s) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITestsDefaultSetUp, startShouldReturnFutureWithExceptionIfStartRequestRefused)
{
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future = driver_->start();
  start_req_received_barrier.waitTillRelease(2s);

  hw_mock_->sendStartReply(data_conversion_layer::scanner_reply::Message::OperationResult::refused);
  EXPECT_FUTURE_IS_READY(start_future, 2s);
  EXPECT_THROW_AND_WHAT(start_future.get(), std::runtime_error, "Start Request refused by device.");
}

TEST_F(ScannerAPITestsDefaultSetUp, stopShouldReturnFutureWithExceptionIfStopRequestRefused)
{
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future = driver_->start();
  start_req_received_barrier.waitTillRelease(2s);

  hw_mock_->sendStartReply(data_conversion_layer::scanner_reply::Message::OperationResult::accepted);
  EXPECT_FUTURE_IS_READY(start_future, 2s) << "Scanner::start() not finished";

  util::Barrier stop_req_received_barrier;
  EXPECT_STOP_REQUEST_CALL(*hw_mock_).WillOnce(OpenBarrier(&stop_req_received_barrier));

  std::future<void> stop_future = driver_->stop();
  stop_req_received_barrier.waitTillRelease(2s);
  hw_mock_->sendStopReply(data_conversion_layer::scanner_reply::Message::OperationResult::refused);
  EXPECT_THROW_AND_WHAT(stop_future.get(), std::runtime_error, "Stop Request refused by device.");
}

TEST_F(ScannerAPITestsDefaultSetUp, startShouldReturnFutureWithExceptionIfUnknownResultSent)
{
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future = driver_->start();
  start_req_received_barrier.waitTillRelease(2s);

  hw_mock_->sendStartReply(data_conversion_layer::scanner_reply::Message::OperationResult::unknown);
  EXPECT_FUTURE_IS_READY(start_future, 2s);
  EXPECT_THROW_AND_WHAT(
      start_future.get(),
      std::runtime_error,
      fmt::format("Unknown result code {:#04x} in start reply.",
                  static_cast<uint32_t>(data_conversion_layer::scanner_reply::Message::OperationResult::unknown))
          .c_str());
}

TEST_F(ScannerAPITestsDefaultSetUp, stopShouldReturnFutureWithExceptionIfUnknownResultSent)
{
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future = driver_->start();
  start_req_received_barrier.waitTillRelease(2s);

  hw_mock_->sendStartReply(data_conversion_layer::scanner_reply::Message::OperationResult::accepted);
  EXPECT_FUTURE_IS_READY(start_future, 2s) << "Scanner::start() not finished";

  util::Barrier stop_req_received_barrier;
  EXPECT_STOP_REQUEST_CALL(*hw_mock_).WillOnce(OpenBarrier(&stop_req_received_barrier));

  std::future<void> stop_future = driver_->stop();
  stop_req_received_barrier.waitTillRelease(2s);
  hw_mock_->sendStopReply(data_conversion_layer::scanner_reply::Message::OperationResult::unknown);
  EXPECT_THROW_AND_WHAT(
      stop_future.get(),
      std::runtime_error,
      fmt::format("Unknown result code {:#04x} in stop reply.",
                  static_cast<uint32_t>(data_conversion_layer::scanner_reply::Message::OperationResult::unknown))
          .c_str());
}

TEST_F(ScannerAPITestsDefaultSetUp, startShouldSucceedDespiteUnexpectedMonitoringFrame)
{
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future;
  EXPECT_NO_BLOCK_NO_THROW(start_future = driver_->start(););

  start_req_received_barrier.waitTillRelease(2s);

  hw_mock_->sendMonitoringFrame(createMonitoringFrameMsg());
  EXPECT_FUTURE_TIMEOUT(start_future, FUTURE_WAIT_TIMEOUT) << "Scanner::start() finished without receiveing reply";

  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, 2s) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITestsDefaultSetUp, shouldSendStopRequestAndValidFutureOnStopCall)
{
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  util::Barrier stop_req_received_barrier;
  EXPECT_STOP_REQUEST_CALL(*hw_mock_).WillOnce(OpenBarrier(&stop_req_received_barrier));

  std::future<void> stop_future;
  EXPECT_NO_BLOCK_NO_THROW(stop_future = driver_->stop(););

  stop_req_received_barrier.waitTillRelease(2s);
  EXPECT_FUTURE_TIMEOUT(stop_future, FUTURE_WAIT_TIMEOUT) << "Scanner::stop() finished without receiveing reply";
  hw_mock_->sendStopReply();
  EXPECT_FUTURE_IS_READY(stop_future, 2s) << "Scanner::stop() not finished";
}

TEST_F(ScannerAPITestsDefaultSetUp, shouldReturnInvalidFutureWhenStopIsCalledSecondTime)
{
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  util::Barrier stop_req_received_barrier;
  EXPECT_STOP_REQUEST_CALL(*hw_mock_).WillOnce(OpenBarrier(&stop_req_received_barrier));

  std::future<void> stop_future;
  EXPECT_NO_BLOCK_NO_THROW(stop_future = driver_->stop(););
  EXPECT_TRUE(stop_future.valid()) << "First call too Scanner::stop() should return VALID std::future";
  for (int i = 0; i < 5; ++i)
  {
    std::future<void> second_stop_future;
    EXPECT_NO_BLOCK_NO_THROW(second_stop_future = driver_->stop(););
    EXPECT_FALSE(second_stop_future.valid())
        << "Subsequenct calls to Scanner::stop() should return INVALID std::future";
  }

  stop_req_received_barrier.waitTillRelease(2s);
  hw_mock_->sendStopReply();
  EXPECT_FUTURE_IS_READY(stop_future, 2s) << "Scanner::stop() not finished";
}

TEST_F(ScannerAPITestsDefaultSetUp, shouldResendStartRequestIfNoStartReplyIsSent)
{
  INJECT_LOG_MOCK;
  EXPECT_ANY_LOG().Times(AnyNumber());

  util::Barrier error_msg_barrier;
  util::Barrier twice_called_barrier;
  {
    InSequence seq;
    EXPECT_CALL(*hw_mock_, receiveControlMsg(_, _)).Times(1);
    EXPECT_CALL(*hw_mock_, receiveControlMsg(_, _)).Times(1).WillOnce(OpenBarrier(&twice_called_barrier));
  }

  EXPECT_LOG_SHORT(INFO, "Scanner: Start scanner called.").Times(1);
  EXPECT_LOG_SHORT(ERROR,
                   "StateMachine: Timeout while waiting for the scanner to start! Retrying... "
                   "(Please check the ethernet connection or contact PILZ support if the error persists.)")
      .Times(AtLeast(1))
      .WillOnce(OpenBarrier(&error_msg_barrier));
  EXPECT_LOG_SHORT(INFO, "ScannerController: Scanner started successfully.").Times(1);

  std::future<void> start_future;
  EXPECT_NO_BLOCK_NO_THROW(start_future = driver_->start(););

  error_msg_barrier.waitTillRelease(2s);
  twice_called_barrier.waitTillRelease(5s);
  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, 2s) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITestsDefaultSetUp, shouldShowInfoWithNewActiveZonesetOnlyWhenItChanges)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const auto msg1{ createMonitoringFrameMsgWithZoneset(2) };
  const auto msg2{ createMonitoringFrameMsgWithZoneset(4) };
  util::Barrier diagnostic_barrier;

  EXPECT_LOG_SHORT(INFO, "Scanner: The scanner switched to active zoneset 2").Times(1);
  EXPECT_LOG_SHORT(INFO, "Scanner: The scanner switched to active zoneset 4")
      .WillOnce(OpenBarrier(&diagnostic_barrier));

  hw_mock_->sendMonitoringFrame(msg1);
  hw_mock_->sendMonitoringFrame(msg1);
  hw_mock_->sendMonitoringFrame(msg2);

  diagnostic_barrier.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITestsDefaultSetUp, shouldShowUserMsgIfMonitoringFrameReceiveTimeout)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: Timeout while waiting for MonitoringFrame message."
                   " (Please check the ethernet connection or contact PILZ support if the error persists.)")
      .WillOnce(OpenBarrier(&user_msg_barrier));

  user_msg_barrier.waitTillRelease(3s);  // the tested code itself has a timeout of 1s

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITestsFragmented, LaserScanShouldContainAllInfosTransferedByMonitoringFrameMsg)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const auto msg{ createMonitoringFrameMsg() };
  util::Barrier monitoring_frame_barrier;
  EXPECT_CALLBACK_WILL_OPEN_BARRIER(user_callbacks_, { msg }, monitoring_frame_barrier);
  util::Barrier diagnostic_barrier;

  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: The scanner reports an error: {Device: Master - Alarm: The front panel of the "
                   "safety "
                   "laser scanner must be cleaned.}")
      .WillOnce(OpenBarrier(&diagnostic_barrier));

  hw_mock_->sendMonitoringFrame(msg);

  monitoring_frame_barrier.waitTillRelease(2s);
  diagnostic_barrier.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITestsFragmented, shouldCallLaserscanCallbackInCaseOfMissingDiagnostics)
{
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  util::Barrier monitoring_frame_barrier;
  EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).WillOnce(OpenBarrier(&monitoring_frame_barrier));

  const auto msg{ createMonitoringFrameMsgWithoutDiagnostics() };
  hw_mock_->sendMonitoringFrame(msg);

  monitoring_frame_barrier.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITestsFragmented, shouldNotCallLaserscanCallbackInCaseOfEmptyMonitoringFrame)
{
  INJECT_LOG_MOCK;
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).Times(0);

  util::Barrier empty_msg_received;
  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: No transition in state \"WaitForMonitoringFrame\" for event "
                   "\"MonitoringFrameReceivedError\".")
      .WillOnce(OpenBarrier(&empty_msg_received));

  hw_mock_->sendEmptyMonitoringFrame();
  empty_msg_received.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITestsFragmented, shouldNotCallLaserscanCallbackInCaseOfMissingMeassurements)
{
  INJECT_LOG_MOCK;
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).Times(0);

  util::Barrier log_msgs_barrier;
  EXPECT_LOG_SHORT(DEBUG,
                   "StateMachine: No measurement data in current monitoring frame(s), skipping laser scan "
                   "callback.")
      .WillOnce(OpenBarrier(&log_msgs_barrier));

  hw_mock_->sendMonitoringFrame(createMonitoringFrameMsg(42, util::TenthOfDegree{ 0 }, util::TenthOfDegree{ 0 }));
  log_msgs_barrier.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITestsFragmented, shouldShowUserMsgIfMonitoringFramesAreMissing)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const std::size_t num_scans_per_round{ 6 };
  auto valid_scan_msgs{ createMonitoringFrameMsgs(41, num_scans_per_round) };
  auto invalid_scan_round_msgs{ createMonitoringFrameMsgs(42, num_scans_per_round - 1) };
  auto new_scan_round_msg{ createMonitoringFrameMsg(43) };

  util::Barrier all_frames_received_barrier;
  {
    InSequence seq;
    EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).Times(num_scans_per_round + num_scans_per_round - 1);
    EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).WillOnce(OpenBarrier(&all_frames_received_barrier));
  }

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN,
                   "ScanBuffer: Detected a MonitoringFrame from a new scan round before the old one was complete."
                   " Dropping the incomplete round."
                   " (Please check the ethernet connection or contact PILZ support if the error persists.)")
      .WillOnce(OpenBarrier(&user_msg_barrier));

  hw_mock_->sendMonitoringFrames(valid_scan_msgs);
  hw_mock_->sendMonitoringFrames(invalid_scan_round_msgs);
  hw_mock_->sendMonitoringFrame(new_scan_round_msg);

  user_msg_barrier.waitTillRelease(2s);
  all_frames_received_barrier.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITestsFragmented, shouldShowUserMsgIfTooManyMonitoringFramesAreReceived)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const std::size_t num_scans_per_round{ 6 };

  auto invalid_scan_round{ createMonitoringFrameMsgs(42, num_scans_per_round + 1) };

  util::Barrier all_frames_received;
  {
    InSequence sec;
    EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).Times((num_scans_per_round));
    EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).WillOnce(OpenBarrier(&all_frames_received));
  }

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN, "ScanBuffer: Received too many MonitoringFrames for one scan round.")
      .WillOnce(OpenBarrier(&user_msg_barrier));

  hw_mock_->sendMonitoringFrames(invalid_scan_round);

  user_msg_barrier.waitTillRelease(2s);
  all_frames_received.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITestsUnfragmented, shouldCallLaserScanCallbackOnlyOneTimeWithAllInformation)
{
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const auto msgs{ createMonitoringFrameMsgsForScanRound(2, 6) };
  util::Barrier monitoring_frame_barrier;
  EXPECT_CALLBACK_WILL_OPEN_BARRIER(user_callbacks_, msgs, monitoring_frame_barrier);

  hw_mock_->sendMonitoringFrames(msgs);

  monitoring_frame_barrier.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITestsUnfragmented, shouldShowOneUserMsgIfFirstTwoScanRoundsStartEarly)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  auto ignored_short_first_round = createMonitoringFrameMsgsForScanRound(2, 1);
  auto accounted_short_round = createMonitoringFrameMsgsForScanRound(3, 5);
  auto valid_round = createMonitoringFrameMsgsForScanRound(4, 6);

  util::Barrier monitoring_frame_barrier;
  EXPECT_CALLBACK_WILL_OPEN_BARRIER(user_callbacks_, valid_round, monitoring_frame_barrier);

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN,
                   "ScanBuffer: Detected a MonitoringFrame from a new scan round before the old one was complete."
                   " Dropping the incomplete round."
                   " (Please check the ethernet connection or contact PILZ support if the error persists.)")
      .WillOnce(OpenBarrier(&user_msg_barrier));

  for (const auto& msgs : { ignored_short_first_round, accounted_short_round, valid_round })
  {
    hw_mock_->sendMonitoringFrames(msgs);
  }

  monitoring_frame_barrier.waitTillRelease(2s);
  user_msg_barrier.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITestsUnfragmented, shouldIgnoreMonitoringFrameOfFormerScanRound)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  auto msg_round2 = createMonitoringFrameMsg(2);
  auto msgs_round3 = createMonitoringFrameMsgsForScanRound(3, 6);

  util::Barrier monitoring_frame_barrier;
  EXPECT_CALLBACK_WILL_OPEN_BARRIER(user_callbacks_, msgs_round3, monitoring_frame_barrier);

  auto last_msg_of_round_3 = msgs_round3.back();
  msgs_round3.pop_back();
  auto first_msgs_of_round_3 = msgs_round3;

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN,
                   "ScanBuffer: Detected a MonitoringFrame from an earlier round. "
                   " The scan round will ignore it.")
      .WillOnce(OpenBarrier(&user_msg_barrier));

  hw_mock_->sendMonitoringFrames(first_msgs_of_round_3);
  hw_mock_->sendMonitoringFrame(msg_round2);
  hw_mock_->sendMonitoringFrame(last_msg_of_round_3);

  monitoring_frame_barrier.waitTillRelease(2s);
  user_msg_barrier.waitTillRelease(2s);

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
