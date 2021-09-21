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
#include <thread>
#include <vector>

// Test frameworks
#include "psen_scan_v2_standalone/communication_layer/scanner_mock.h"
#include "psen_scan_v2_standalone/util/integrationtest_helper.h"
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

class ScannerAPITests : public testing::Test
{
protected:
  void SetUp() override;
  void setUpScannerConfig(const std::string& host_ip = HOST_IP_ADDRESS, bool fragmented = FRAGMENTED_SCAN);
  ScannerConfiguration generateScannerConfig(const std::string& host_ip, bool fragmented);
  void setUpScannerV2();
  void setUpNiceScannerMock();
  void setUpStrictScannerMock();
  void prepareScannerMockStartReply();
  void startScanner();
  std::unique_ptr<util::Barrier> prepareStartRequestBarrier(const ScannerConfiguration& config);
  std::unique_ptr<util::Barrier>
  prepareMonitoringFrameBarrier(const std::vector<data_conversion_layer::monitoring_frame::Message>& msgs);
  void expectNotDoneYet(const std::string& method_name, const std::future<void>& future);
  void expectIsDone(const std::string& method_name, const std::future<void>& future);
  void sendMonitoringFrames(const std::vector<data_conversion_layer::monitoring_frame::Message>& msgs);

protected:
  const PortHolder port_holder_{ ++GLOBAL_PORT_HOLDER };
  std::unique_ptr<NiceMock<ScannerMock>> nice_scanner_mock_;
  std::unique_ptr<StrictMock<ScannerMock>> strict_scanner_mock_;
  std::unique_ptr<ScannerConfiguration> config_;
  UserCallbacks user_callbacks_;
  std::unique_ptr<ScannerV2> scanner_;
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

void ScannerAPITests::setUpScannerV2()
{
  scanner_.reset(
      new ScannerV2(*config_, std::bind(&UserCallbacks::LaserScanCallback, &user_callbacks_, std::placeholders::_1)));
}

void ScannerAPITests::setUpNiceScannerMock()
{
  nice_scanner_mock_.reset(new NiceMock<ScannerMock>{ HOST_IP_ADDRESS, port_holder_ });
}

void ScannerAPITests::setUpStrictScannerMock()
{
  strict_scanner_mock_.reset(new StrictMock<ScannerMock>{ HOST_IP_ADDRESS, port_holder_ });
}

void ScannerAPITests::prepareScannerMockStartReply()
{
  if (strict_scanner_mock_)
  {
    EXPECT_CALL(*strict_scanner_mock_,
                receiveControlMsg(_,
                                  data_conversion_layer::start_request::serialize(
                                      data_conversion_layer::start_request::Message(*config_))))
        .WillOnce(InvokeWithoutArgs([this]() { strict_scanner_mock_->sendStartReply(); }));
  }
  else
  {
    ON_CALL(*nice_scanner_mock_,
            receiveControlMsg(_,
                              data_conversion_layer::start_request::serialize(
                                  data_conversion_layer::start_request::Message(*config_))))
        .WillByDefault(InvokeWithoutArgs([this]() { nice_scanner_mock_->sendStartReply(); }));
  }
}

std::unique_ptr<util::Barrier> ScannerAPITests::prepareStartRequestBarrier(const ScannerConfiguration& config)
{
  const data_conversion_layer::start_request::Message start_req(config);
  auto start_req_received_barrier = std::make_unique<util::Barrier>();

  if (strict_scanner_mock_)
  {
    EXPECT_CALL(*strict_scanner_mock_, receiveControlMsg(_, data_conversion_layer::start_request::serialize(start_req)))
        .WillOnce(OpenBarrier(start_req_received_barrier.get()));
  }
  else
  {
    ON_CALL(*nice_scanner_mock_, receiveControlMsg(_, data_conversion_layer::start_request::serialize(start_req)))
        .WillByDefault(OpenBarrier(start_req_received_barrier.get()));
  }
  return start_req_received_barrier;
}

std::unique_ptr<util::Barrier> ScannerAPITests::prepareMonitoringFrameBarrier(
    const std::vector<data_conversion_layer::monitoring_frame::Message>& msgs)
{
  const auto timestamp{ util::getCurrentTime() };
  const auto scan{ createReferenceScan(msgs, timestamp) };

  auto monitoring_frame_barrier = std::make_unique<util::Barrier>();

  EXPECT_CALL(user_callbacks_,
              LaserScanCallback(AllOf(ScanDataEqual(scan), TimestampInExpectedTimeframe(scan, timestamp))))
      .WillOnce(OpenBarrier(monitoring_frame_barrier.get()));
  return monitoring_frame_barrier;
}

void ScannerAPITests::startScanner()
{
  prepareScannerMockStartReply();
  if (strict_scanner_mock_)
  {
    strict_scanner_mock_->startListeningForControlMsg();
  }
  else
  {
    nice_scanner_mock_->startListeningForControlMsg();
  }
  const auto start_future = scanner_->start();
  expectIsDone("Scanner::start()", start_future);
}

void ScannerAPITests::expectNotDoneYet(const std::string& method_name, const std::future<void>& future)
{
  EXPECT_EQ(future.wait_for(FUTURE_WAIT_TIMEOUT), std::future_status::timeout) << method_name << " finished to early";
}

void ScannerAPITests::expectIsDone(const std::string& method_name, const std::future<void>& future)
{
  EXPECT_EQ(future.wait_for(DEFAULT_TIMEOUT), std::future_status::ready) << method_name << " not finished";
}

void ScannerAPITests::sendMonitoringFrames(const std::vector<data_conversion_layer::monitoring_frame::Message>& msgs)
{
  for (const auto& msg : msgs)
  {
    if (strict_scanner_mock_)
    {
      strict_scanner_mock_->sendMonitoringFrame(msg);
    }
    else
    {
      nice_scanner_mock_->sendMonitoringFrame(msg);
    }
    // Sleep to ensure that message are not sent too fast which might cause messages overwrite in socket buffer
    std::this_thread::sleep_for(10ms);
  }
}

TEST_F(ScannerAPITests, testStartFunctionality)
{
  setUpScannerConfig();
  setUpScannerV2();
  setUpStrictScannerMock();
  const auto start_req_received_barrier = prepareStartRequestBarrier(*config_);

  strict_scanner_mock_->startListeningForControlMsg();
  const auto start_future = scanner_->start();

  ASSERT_TRUE(start_req_received_barrier->waitTillRelease(DEFAULT_TIMEOUT)) << "Start request not received";
  expectNotDoneYet("Scanner::start()", start_future);
  strict_scanner_mock_->sendStartReply();
  expectIsDone("Scanner::start()", start_future);
}

TEST_F(ScannerAPITests, shouldReceiveStartRequestWithCorrectHostIpWhenUsingAutoInConfigAsHostIp)
{
  setUpScannerConfig("auto");
  setUpScannerV2();
  setUpStrictScannerMock();
  const auto start_req_received_barrier =
      prepareStartRequestBarrier(generateScannerConfig(HOST_IP_ADDRESS, FRAGMENTED_SCAN));

  strict_scanner_mock_->startListeningForControlMsg();
  const auto start_future = scanner_->start();

  ASSERT_TRUE(start_req_received_barrier->waitTillRelease(DEFAULT_TIMEOUT)) << "Start request not received";
  strict_scanner_mock_->sendStartReply();
  expectIsDone("Scanner::start()", start_future);
}

TEST_F(ScannerAPITests, shouldReturnInvalidFutureWhenStartIsCalledSecondTime)
{
  setUpScannerConfig();
  setUpScannerV2();
  setUpNiceScannerMock();

  nice_scanner_mock_->startListeningForControlMsg();
  const auto start_future = scanner_->start();
  EXPECT_TRUE(start_future.valid()) << "First call too Scanner::start() should return VALID std::future";
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_FALSE(scanner_->start().valid()) << "Subsequenct calls to Scanner::start() should return INVALID "
                                               "std::future";
  }
  nice_scanner_mock_->sendStartReply();
  expectIsDone("Scanner::start()", start_future);
}

TEST_F(ScannerAPITests, startShouldSucceedDespiteUnexpectedMonitoringFrame)
{
  setUpScannerConfig();
  setUpScannerV2();
  setUpNiceScannerMock();

  auto start_req_received_barrier = prepareStartRequestBarrier(*config_);

  nice_scanner_mock_->startListeningForControlMsg();
  const auto start_future = scanner_->start();

  ASSERT_TRUE(start_req_received_barrier->waitTillRelease(DEFAULT_TIMEOUT)) << "Start request not received";

  nice_scanner_mock_->sendMonitoringFrame(createValidMonitoringFrameMsg());
  expectNotDoneYet("Scanner::start()", start_future);

  nice_scanner_mock_->sendStartReply();
  expectIsDone("Scanner::start()", start_future);
}

TEST_F(ScannerAPITests, testStopFunctionality)
{
  setUpScannerConfig();
  setUpScannerV2();
  setUpStrictScannerMock();
  startScanner();

  util::Barrier stop_req_received_barrier;
  EXPECT_CALL(*strict_scanner_mock_, receiveControlMsg(_, data_conversion_layer::stop_request::serialize()))
      .WillOnce(OpenBarrier(&stop_req_received_barrier));

  strict_scanner_mock_->startListeningForControlMsg();
  const auto stop_future{ std::async(std::launch::async, [this]() {
    const auto stop_future = scanner_->stop();
    stop_future.wait();
  }) };

  EXPECT_TRUE(stop_req_received_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Stop request not received";
  expectNotDoneYet("Scanner::stop()", stop_future);
  strict_scanner_mock_->sendStopReply();
  expectIsDone("Scanner::stop()", stop_future);
}

TEST_F(ScannerAPITests, shouldReturnInvalidFutureWhenStopIsCalledSecondTime)
{
  setUpScannerConfig();
  setUpScannerV2();
  setUpNiceScannerMock();
  startScanner();

  nice_scanner_mock_->startListeningForControlMsg();
  const auto stop_future = scanner_->stop();
  EXPECT_TRUE(stop_future.valid()) << "First call too Scanner::stop() should return VALID std::future";
  for (int i = 0; i < 5; ++i)
  {
    EXPECT_FALSE(scanner_->stop().valid()) << "Subsequenct calls to Scanner::stop() should return INVALID std::future";
  }

  nice_scanner_mock_->sendStopReply();
  expectIsDone("Scanner::stop()", stop_future);
}

TEST_F(ScannerAPITests, testStartReplyTimeout)
{
  INJECT_LOG_MOCK;
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2();
  setUpStrictScannerMock();

  util::Barrier error_msg_barrier;
  util::Barrier twice_called_barrier;
  {
    InSequence seq;
    EXPECT_CALL(*strict_scanner_mock_, receiveControlMsg(_, _)).Times(1);
    EXPECT_CALL(*strict_scanner_mock_, receiveControlMsg(_, _)).Times(1).WillOnce(OpenBarrier(&twice_called_barrier));
  }

  EXPECT_LOG_SHORT(INFO, "Scanner: Start scanner called.").Times(1);
  EXPECT_LOG_SHORT(ERROR,
                   "StateMachine: Timeout while waiting for the scanner to start! Retrying... "
                   "(Please check the ethernet connection or contact PILZ support if the error persists.)")
      .Times(AtLeast(1))
      .WillOnce(OpenBarrier(&error_msg_barrier));
  EXPECT_LOG_SHORT(INFO, "ScannerController: Scanner started successfully.").Times(1);

  strict_scanner_mock_->startContinuousListeningForControlMsg();
  const auto start_future{ std::async(std::launch::async, [this]() {
    const auto scanner_start = scanner_->start();
    scanner_start.wait();
  }) };

  EXPECT_TRUE(error_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Error message not received";
  EXPECT_TRUE(twice_called_barrier.waitTillRelease(5000ms)) << "Start reply not send at least twice in time";
  strict_scanner_mock_->sendStartReply();
  expectIsDone("Scanner::start()", start_future);
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, LaserScanShouldContainAllInfosTransferedByMonitoringFrameMsg)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2();
  setUpStrictScannerMock();
  startScanner();

  const auto msg{ createValidMonitoringFrameMsg() };
  auto monitoring_frame_barrier = prepareMonitoringFrameBarrier({ msg });
  util::Barrier diagnostic_barrier;

  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: The scanner reports an error: {Device: Master - Alarm: The front panel of the "
                   "safety "
                   "laser scanner must be cleaned.}")
      .Times(1)
      .WillOnce(OpenBarrier(&diagnostic_barrier));

  strict_scanner_mock_->sendMonitoringFrame(msg);

  EXPECT_TRUE(monitoring_frame_barrier->waitTillRelease(DEFAULT_TIMEOUT)) << "Monitoring frame not received";
  EXPECT_TRUE(diagnostic_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Diagnostic message not received";
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldCallLaserScanCBOnlyOneTimeWithAllInformationWhenUnfragmentedScanIsEnabled)
{
  setUpScannerConfig(HOST_IP_ADDRESS, UNFRAGMENTED_SCAN);
  setUpScannerV2();
  setUpNiceScannerMock();
  startScanner();

  const auto msgs{ createMonitoringFrameMsgsForScanRound(2, 6) };
  auto monitoring_frame_barrier = prepareMonitoringFrameBarrier(msgs);

  sendMonitoringFrames(msgs);

  EXPECT_TRUE(monitoring_frame_barrier->waitTillRelease(DEFAULT_TIMEOUT)) << "Monitoring frame not received";
}

TEST_F(ScannerAPITests, shouldShowOneUserMsgIfFirstTwoScanRoundsStartEarly)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig(HOST_IP_ADDRESS, UNFRAGMENTED_SCAN);
  setUpScannerV2();
  setUpNiceScannerMock();
  startScanner();

  std::vector<psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::Message> ignored_short_first_round =
      createMonitoringFrameMsgsForScanRound(2, 1);
  std::vector<psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::Message> accounted_short_round =
      createMonitoringFrameMsgsForScanRound(3, 5);
  std::vector<psen_scan_v2_standalone::data_conversion_layer::monitoring_frame::Message> valid_round =
      createMonitoringFrameMsgsForScanRound(4, 6);

  auto monitoring_frame_barrier = prepareMonitoringFrameBarrier(valid_round);

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN,
                   "ScanBuffer: Detected a MonitoringFrame from a new scan round before the old one was complete."
                   " Dropping the incomplete round."
                   " (Please check the ethernet connection or contact PILZ support if the error persists.)")
      .Times(1)
      .WillOnce(OpenBarrier(&user_msg_barrier));

  for (const auto& msgs : { ignored_short_first_round, accounted_short_round, valid_round })
  {
    sendMonitoringFrames(msgs);
  }

  EXPECT_TRUE(monitoring_frame_barrier->waitTillRelease(DEFAULT_TIMEOUT)) << "Monitoring frame not received";
  EXPECT_TRUE(user_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT))
      << "Monitoring frame of new scan round not recognized ";
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldIgnoreMonitoringFrameOfFormerScanRound)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig(HOST_IP_ADDRESS, UNFRAGMENTED_SCAN);
  setUpScannerV2();
  setUpNiceScannerMock();
  startScanner();

  auto msg_round2 = createValidMonitoringFrameMsg(2);
  auto msgs_round3 = createMonitoringFrameMsgsForScanRound(3, 6);

  auto monitoring_frame_barrier = prepareMonitoringFrameBarrier(msgs_round3);

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN,
                   "ScanBuffer: Detected a MonitoringFrame from an earlier round. "
                   " The scan round will ignore it.")
      .Times(1)
      .WillOnce(OpenBarrier(&user_msg_barrier));

  sendMonitoringFrames(msgs_round3);
  nice_scanner_mock_->sendMonitoringFrame(msg_round2);
  nice_scanner_mock_->sendMonitoringFrame(msgs_round3.back());

  EXPECT_TRUE(monitoring_frame_barrier->waitTillRelease(DEFAULT_TIMEOUT)) << "Monitoring frame not received";
  EXPECT_TRUE(user_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Dropped Monitoring frame not recognized";
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldNotCallLaserscanCallbackInCaseOfEmptyMonitoringFrame)
{
  INJECT_NICE_LOG_MOCK;
  setUpScannerConfig();
  setUpScannerV2();
  setUpNiceScannerMock();
  startScanner();

  util::Barrier valid_msg_barrier;
  EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).WillOnce(OpenBarrier(&valid_msg_barrier));

  util::Barrier empty_msg_received;
  // Needed to allow all other log messages which might be received
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: No transition in state \"WaitForMonitoringFrame\" for event "
                   "\"MonitoringFrameReceivedError\".")
      .Times(1)
      .WillOnce(OpenBarrier(&empty_msg_received));

  std::cout << "ScannerAPITests: Send empty monitoring frame..." << std::endl;
  nice_scanner_mock_->sendEmptyMonitoringFrame();
  EXPECT_TRUE(empty_msg_received.waitTillRelease(DEFAULT_TIMEOUT)) << "Empty monitoring frame not received";

  std::cout << "ScannerAPITests: Send valid monitoring frame..." << std::endl;
  nice_scanner_mock_->sendMonitoringFrame(createValidMonitoringFrameMsg());
  EXPECT_TRUE(valid_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Valid monitoring frame not received";
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldNotCallLaserscanCallbackInCaseOfMissingMeassurements)
{
  INJECT_NICE_LOG_MOCK;
  setUpScannerConfig();
  setUpScannerV2();
  setUpNiceScannerMock();
  startScanner();

  util::Barrier valid_msg_barrier;
  EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).Times(0);

  // Needed to allow all other log messages which might be received
  EXPECT_ANY_LOG().Times(AnyNumber());
  EXPECT_LOG_SHORT(DEBUG,
                   "StateMachine: No measurement data in current monitoring frame(s), skipping laser scan "
                   "callback.")
      .Times(1)
      .WillOnce(OpenBarrier(&valid_msg_barrier));

  std::cout << "ScannerAPITests: Send monitoring frame without measurement data ..." << std::endl;
  nice_scanner_mock_->sendMonitoringFrame(
      createValidMonitoringFrameMsg(42, util::TenthOfDegree{ 0 }, util::TenthOfDegree{ 0 }));
  EXPECT_TRUE(valid_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "Valid monitoring frame not received";
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
  setUpScannerV2();
  setUpNiceScannerMock();
  startScanner();

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

  EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).Times(num_scans_per_round + (num_scans_per_round - 1) + 1);

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN,
                   "ScanBuffer: Detected a MonitoringFrame from a new scan round before the old one was complete."
                   " Dropping the incomplete round."
                   " (Please check the ethernet connection or contact PILZ support if the error persists.)")
      .Times(1)
      .WillOnce(OpenBarrier(&user_msg_barrier));

  sendMonitoringFrames(valid_scan_round_msgs);
  sendMonitoringFrames(invalid_scan_round_msgs);

  EXPECT_TRUE(user_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "User message not received";
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldShowUserMsgIfTooManyMonitoringFramesAreReceived)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2();
  setUpNiceScannerMock();
  startScanner();

  const std::size_t num_scans_per_round{ 6 };

  const uint32_t scan_counter{ 42 };
  std::vector<data_conversion_layer::monitoring_frame::Message> msgs{ createValidMonitoringFrameMsgs(
      scan_counter, num_scans_per_round + 1) };
  msgs.emplace_back(createValidMonitoringFrameMsg(scan_counter + 1));

  EXPECT_CALL(user_callbacks_, LaserScanCallback(_)).Times((num_scans_per_round + 1) + 1);

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN, "ScanBuffer: Received too many MonitoringFrames for one scan round.")
      .Times(1)
      .WillOnce(OpenBarrier(&user_msg_barrier));

  sendMonitoringFrames(msgs);

  EXPECT_TRUE(user_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "User message not received";
  REMOVE_LOG_MOCK
}

TEST_F(ScannerAPITests, shouldShowUserMsgIfMonitoringFrameReceiveTimeout)
{
  INJECT_LOG_MOCK
  EXPECT_ANY_LOG().Times(AnyNumber());
  setUpScannerConfig();
  setUpScannerV2();
  setUpNiceScannerMock();
  startScanner();

  util::Barrier user_msg_barrier;
  EXPECT_LOG_SHORT(WARN,
                   "StateMachine: Timeout while waiting for MonitoringFrame message."
                   " (Please check the ethernet connection or contact PILZ support if the error persists.)")
      .Times(1)
      .WillOnce(OpenBarrier(&user_msg_barrier));

  EXPECT_TRUE(user_msg_barrier.waitTillRelease(DEFAULT_TIMEOUT)) << "User message not received";
  REMOVE_LOG_MOCK
}

}  // namespace psen_scan_v2_standalone_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
