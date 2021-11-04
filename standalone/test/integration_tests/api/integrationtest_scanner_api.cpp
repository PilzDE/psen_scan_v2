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
#include "psen_scan_v2_standalone/util/expectations.h"
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
static constexpr std::chrono::seconds DEFAULT_TIMEOUT{ 3 };

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
    EXPECT_CALL(cb, LaserScanCallback(AllOf(ScanDataEqual(scan), TimestampInExpectedTimeframe(scan, timestamp))))      \
        .WillOnce(OpenBarrier(&barrier));                                                                              \
  } while (false)

#define EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock, driver, config)                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    util::Barrier start_req_barrier;                                                                                   \
    std::future<void> start_future;                                                                                    \
    EXPECT_START_REQUEST_CALL(*hw_mock, *config).WillOnce(OpenBarrier(&start_req_barrier));                            \
    EXPECT_DOES_NOT_BLOCK(start_future = driver->start(););                                                            \
    EXPECT_BARRIER_OPENS(start_req_barrier, DEFAULT_TIMEOUT) << "Start request not received";                          \
    hw_mock->sendStartReply();                                                                                         \
    EXPECT_FUTURE_IS_READY(start_future, DEFAULT_TIMEOUT) << "Scanner::start() not finished";                          \
  } while (false)

#define EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock, driver)                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    util::Barrier stop_req_barrier;                                                                                    \
    std::future<void> stop_future;                                                                                     \
    EXPECT_STOP_REQUEST_CALL(*hw_mock).WillOnce(OpenBarrier(&stop_req_barrier));                                       \
    EXPECT_DOES_NOT_BLOCK(stop_future = driver->stop(););                                                              \
    EXPECT_BARRIER_OPENS(stop_req_barrier, DEFAULT_TIMEOUT) << "Stop request not received";                            \
    hw_mock->sendStopReply();                                                                                          \
    EXPECT_FUTURE_IS_READY(stop_future, DEFAULT_TIMEOUT) << "Scanner::stop() not finished";                            \
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
  auto config_builder = ScannerConfigurationBuilder()
                            .hostIP(host_ip)
                            .hostDataPort(port_holder_.data_port_host)
                            .hostControlPort(port_holder_.control_port_host)
                            .scannerIp(SCANNER_IP_ADDRESS)
                            .scannerDataPort(port_holder_.data_port_scanner)
                            .scannerControlPort(port_holder_.control_port_scanner)
                            .scanRange(DEFAULT_SCAN_RANGE)
                            .scanResolution(DEFAULT_SCAN_RESOLUTION)
                            .enableIntensities()
                            .enableFragmentedScans(fragmented);
  return config_builder.build();
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

TEST_F(ScannerAPITests, shouldSendStartRequestAndReturnValidFutureWhenLaunchingWithValidConfig)
{
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future;
  EXPECT_DOES_NOT_BLOCK(start_future = driver_->start(););

  EXPECT_BARRIER_OPENS(start_req_received_barrier, DEFAULT_TIMEOUT) << "Start request not received";
  EXPECT_FUTURE_TIMEOUT(start_future, FUTURE_WAIT_TIMEOUT) << "Scanner::start() finished without receiveing reply";
  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, DEFAULT_TIMEOUT) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITests, shouldReceiveStartRequestWithCorrectHostIpWhenUsingAutoInConfigAsHostIp)
{
  setUpScannerConfig("auto");
  setUpScannerV2Driver();
  setUpScannerHwMock();
  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, generateScannerConfig(HOST_IP_ADDRESS, FRAGMENTED_SCAN))
      .WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future;
  EXPECT_DOES_NOT_BLOCK(start_future = driver_->start(););

  EXPECT_BARRIER_OPENS(start_req_received_barrier, DEFAULT_TIMEOUT) << "Start request not received";
  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, DEFAULT_TIMEOUT) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITests, shouldReturnInvalidFutureWhenStartIsCalledSecondTime)
{
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();

  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future;
  EXPECT_DOES_NOT_BLOCK(start_future = driver_->start(););
  EXPECT_TRUE(start_future.valid()) << "First call too Scanner::start() should return VALID std::future";
  for (int i = 0; i < 5; ++i)
  {
    std::future<void> second_start_future;
    EXPECT_DOES_NOT_BLOCK(second_start_future = driver_->start(););
    EXPECT_FALSE(second_start_future.valid()) << "Subsequenct calls to Scanner::start() should return INVALID "
                                                 "std::future";
  }
  EXPECT_BARRIER_OPENS(start_req_received_barrier, DEFAULT_TIMEOUT) << "Start request not received";
  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, DEFAULT_TIMEOUT) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITests, startShouldReturnFutureWithExceptionIfStartRequestRefused)
{
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();

  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future = driver_->start();
  start_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT);

  hw_mock_->sendStartReply(data_conversion_layer::scanner_reply::Message::OperationResult::refused);
  EXPECT_FUTURE_IS_READY(start_future, DEFAULT_TIMEOUT);
  EXPECT_THROW_AND_WHAT(start_future.get(), std::runtime_error, "Request refused by device.");
}

TEST_F(ScannerAPITests, startShouldReturnFutureWithExceptionIfUnknownResultSent)
{
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();

  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future = driver_->start();
  start_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT);

  hw_mock_->sendStartReply(data_conversion_layer::scanner_reply::Message::OperationResult::unknown);
  EXPECT_FUTURE_IS_READY(start_future, DEFAULT_TIMEOUT);
  EXPECT_THROW_AND_WHAT(
      start_future.get(),
      std::runtime_error,
      fmt::format("Unknown result code {:#04x} in start reply.",
                  static_cast<uint32_t>(data_conversion_layer::scanner_reply::Message::OperationResult::unknown))
          .c_str());
}

TEST_F(ScannerAPITests, startShouldSucceedDespiteUnexpectedMonitoringFrame)
{
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();

  util::Barrier start_req_received_barrier;
  EXPECT_START_REQUEST_CALL(*hw_mock_, *config_).WillOnce(OpenBarrier(&start_req_received_barrier));

  std::future<void> start_future;
  EXPECT_DOES_NOT_BLOCK(start_future = driver_->start(););

  EXPECT_BARRIER_OPENS(start_req_received_barrier, DEFAULT_TIMEOUT) << "Start request not received";

  hw_mock_->sendMonitoringFrame(createValidMonitoringFrameMsg());
  EXPECT_FUTURE_TIMEOUT(start_future, FUTURE_WAIT_TIMEOUT) << "Scanner::start() finished without receiveing reply";

  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, DEFAULT_TIMEOUT) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITests, shouldSendStopRequestAndValidFutureOnStopCall)
{
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  util::Barrier stop_req_received_barrier;
  EXPECT_STOP_REQUEST_CALL(*hw_mock_).WillOnce(OpenBarrier(&stop_req_received_barrier));

  std::future<void> stop_future;
  EXPECT_DOES_NOT_BLOCK(stop_future = driver_->stop(););

  EXPECT_BARRIER_OPENS(stop_req_received_barrier, DEFAULT_TIMEOUT) << "Stop request not received";
  EXPECT_FUTURE_TIMEOUT(stop_future, FUTURE_WAIT_TIMEOUT) << "Scanner::stop() finished without receiveing reply";
  hw_mock_->sendStopReply();
  EXPECT_FUTURE_IS_READY(stop_future, DEFAULT_TIMEOUT) << "Scanner::stop() not finished";
}

TEST_F(ScannerAPITests, shouldReturnInvalidFutureWhenStopIsCalledSecondTime)
{
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  util::Barrier stop_req_received_barrier;
  EXPECT_STOP_REQUEST_CALL(*hw_mock_).WillOnce(OpenBarrier(&stop_req_received_barrier));

  std::future<void> stop_future;
  EXPECT_DOES_NOT_BLOCK(stop_future = driver_->stop(););
  EXPECT_TRUE(stop_future.valid()) << "First call too Scanner::stop() should return VALID std::future";
  for (int i = 0; i < 5; ++i)
  {
    std::future<void> second_stop_future;
    EXPECT_DOES_NOT_BLOCK(second_stop_future = driver_->stop(););
    EXPECT_FALSE(second_stop_future.valid())
        << "Subsequenct calls to Scanner::stop() should return INVALID std::future";
  }

  EXPECT_BARRIER_OPENS(stop_req_received_barrier, DEFAULT_TIMEOUT) << "Stop request not received";
  hw_mock_->sendStopReply();
  EXPECT_FUTURE_IS_READY(stop_future, DEFAULT_TIMEOUT) << "Scanner::stop() not finished";
}

TEST_F(ScannerAPITests, shouldResendStartRequestIfNoStartReplyIsSent)
{
  INJECT_LOG_MOCK;
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();

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
  EXPECT_DOES_NOT_BLOCK(start_future = driver_->start(););

  EXPECT_BARRIER_OPENS(error_msg_barrier, DEFAULT_TIMEOUT) << "Error message not received";
  EXPECT_BARRIER_OPENS(twice_called_barrier, 5000ms) << "Start reply not send at least twice in time";
  hw_mock_->sendStartReply();
  EXPECT_FUTURE_IS_READY(start_future, DEFAULT_TIMEOUT) << "Scanner::start() not finished";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, LaserScanShouldContainAllInfosTransferedByMonitoringFrameMsg)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const auto msg{ createValidMonitoringFrameMsg() };
  util::Barrier monitoring_frame_barrier;
  EXPECT_CALLBACK_WILL_OPEN_BARRIER(user_callbacks_, { msg }, monitoring_frame_barrier);
  util::Barrier diagnostic_barrier;

  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: The scanner reports an error: {Device: Master - Alarm: The front panel of the "
                   "safety "
                   "laser scanner must be cleaned.}")
      .WillOnce(OpenBarrier(&diagnostic_barrier));

  hw_mock_->sendMonitoringFrame(msg);

  EXPECT_BARRIER_OPENS(monitoring_frame_barrier, DEFAULT_TIMEOUT) << "Monitoring frame not received";
  EXPECT_BARRIER_OPENS(diagnostic_barrier, DEFAULT_TIMEOUT) << "Diagnostic message not received";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldShowInfoWithNewActiveZonesetOnlyWhenItChanges)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const auto msg1{ createValidMonitoringFrameMsgWithZoneset(2) };
  const auto msg2{ createValidMonitoringFrameMsgWithZoneset(4) };
  util::Barrier diagnostic_barrier;

  EXPECT_LOG_SHORT(INFO, "Scanner: The scanner switched to active zoneset 2").Times(1);
  EXPECT_LOG_SHORT(INFO, "Scanner: The scanner switched to active zoneset 4")
      .WillOnce(OpenBarrier(&diagnostic_barrier));

  hw_mock_->sendMonitoringFrame(msg1);
  hw_mock_->sendMonitoringFrame(msg1);
  hw_mock_->sendMonitoringFrame(msg2);

  EXPECT_BARRIER_OPENS(diagnostic_barrier, DEFAULT_TIMEOUT) << "Diagnostic message not received";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldCallLaserScanCallbackOnlyOneTimeWithAllInformationWhenUnfragmentedScanIsEnabled)
{
  setUpScannerConfig(HOST_IP_ADDRESS, UNFRAGMENTED_SCAN);
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const auto msgs{ createMonitoringFrameMsgsForScanRound(2, 6) };
  util::Barrier monitoring_frame_barrier;
  EXPECT_CALLBACK_WILL_OPEN_BARRIER(user_callbacks_, msgs, monitoring_frame_barrier);

  hw_mock_->sendMonitoringFrames(msgs);

  EXPECT_BARRIER_OPENS(monitoring_frame_barrier, DEFAULT_TIMEOUT) << "Monitoring frame not received";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
}

TEST_F(ScannerAPITests, shouldShowOneUserMsgIfFirstTwoScanRoundsStartEarly)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig(HOST_IP_ADDRESS, UNFRAGMENTED_SCAN);
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  std::vector<psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::Message> ignored_short_first_round =
      createMonitoringFrameMsgsForScanRound(2, 1);
  std::vector<psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::Message> accounted_short_round =
      createMonitoringFrameMsgsForScanRound(3, 5);
  std::vector<psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::Message> valid_round =
      createMonitoringFrameMsgsForScanRound(4, 6);

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

  EXPECT_BARRIER_OPENS(monitoring_frame_barrier, DEFAULT_TIMEOUT) << "Monitoring frame not received";
  EXPECT_BARRIER_OPENS(user_msg_barrier, DEFAULT_TIMEOUT) << "Monitoring frame of new scan round not recognized ";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldIgnoreMonitoringFrameOfFormerScanRound)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig(HOST_IP_ADDRESS, UNFRAGMENTED_SCAN);
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  auto msg_round2 = createValidMonitoringFrameMsg(2);
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

  EXPECT_BARRIER_OPENS(monitoring_frame_barrier, DEFAULT_TIMEOUT) << "Monitoring frame not received";
  EXPECT_BARRIER_OPENS(user_msg_barrier, DEFAULT_TIMEOUT) << "Dropped Monitoring frame not recognized";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldNotCallLaserscanCallbackInCaseOfEmptyMonitoringFrame)
{
  INJECT_NICE_LOG_MOCK;
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).Times(0);

  util::Barrier empty_msg_received;
  // Needed to allow all other log messages which might be received
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: No transition in state \"WaitForMonitoringFrame\" for event "
                   "\"MonitoringFrameReceivedError\".")
      .WillOnce(OpenBarrier(&empty_msg_received));

  hw_mock_->sendEmptyMonitoringFrame();
  EXPECT_BARRIER_OPENS(empty_msg_received, DEFAULT_TIMEOUT) << "Empty monitoring frame not received";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldNotCallLaserscanCallbackInCaseOfMissingMeassurements)
{
  INJECT_NICE_LOG_MOCK;
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).Times(0);

  util::Barrier log_msgs_barrier;
  // Needed to allow all other log messages which might be received
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_LOG_SHORT(DEBUG,
                   "StateMachine: No measurement data in current monitoring frame(s), skipping laser scan "
                   "callback.")
      .WillOnce(OpenBarrier(&log_msgs_barrier));

  hw_mock_->sendMonitoringFrame(createValidMonitoringFrameMsg(42, util::TenthOfDegree{ 0 }, util::TenthOfDegree{ 0 }));
  EXPECT_BARRIER_OPENS(log_msgs_barrier, DEFAULT_TIMEOUT) << "Valid monitoring frame not received";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldThrowWhenConstructedWithInvalidLaserScanCallback)
{
  setUpScannerConfig();
  EXPECT_THROW(ScannerV2 scanner(*config_, nullptr);, std::invalid_argument);
}

TEST_F(ScannerAPITests, shouldShowUserMsgIfMonitoringFramesAreMissing)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const std::size_t num_scans_per_round{ 6 };
  auto valid_scan_msgs{ createValidMonitoringFrameMsgs(41, num_scans_per_round) };
  auto invalid_scan_round_msgs{ createValidMonitoringFrameMsgs(42, num_scans_per_round - 1) };
  auto new_scan_round_msg{ createValidMonitoringFrameMsg(43) };

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

  EXPECT_BARRIER_OPENS(user_msg_barrier, DEFAULT_TIMEOUT) << "User message not received";
  EXPECT_BARRIER_OPENS(all_frames_received_barrier, DEFAULT_TIMEOUT) << "Not all frames received";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldShowUserMsgIfTooManyMonitoringFramesAreReceived)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  const std::size_t num_scans_per_round{ 6 };

  auto invalid_scan_round{ createValidMonitoringFrameMsgs(42, num_scans_per_round + 1) };

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

  EXPECT_BARRIER_OPENS(user_msg_barrier, DEFAULT_TIMEOUT) << "User message not received";
  EXPECT_BARRIER_OPENS(all_frames_received, DEFAULT_TIMEOUT) << "Not all frames received";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldShowUserMsgIfMonitoringFrameReceiveTimeout)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2Driver();
  setUpScannerHwMock();
  EXPECT_SCANNER_TO_START_SUCCESSFULLY(hw_mock_, driver_, config_);

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: Timeout while waiting for MonitoringFrame message."
                   " (Please check the ethernet connection or contact PILZ support if the error persists.)")
      .WillOnce(OpenBarrier(&user_msg_barrier));

  EXPECT_BARRIER_OPENS(user_msg_barrier, DEFAULT_TIMEOUT) << "User message not received";

  EXPECT_SCANNER_TO_STOP_SUCCESSFULLY(hw_mock_, driver_);
  REMOVE_LOG_MOCK
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
