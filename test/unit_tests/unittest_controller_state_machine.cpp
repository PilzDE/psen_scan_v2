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

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "psen_scan_v2/controller_state_machine.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/raw_processing.h"

#include "psen_scan_v2/raw_data_array_conversion.h"
#include "psen_scan_v2/udp_frame_dumps.h"

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
class ControllerMock
{
public:
  MOCK_METHOD1(monitoring_frame_cb, void(const MonitoringFrameMsg&));
  MOCK_METHOD0(start_request_cb, void());
  MOCK_METHOD0(stop_request_cb, void());
  MOCK_METHOD0(started_cb, void());
  MOCK_METHOD0(stopped_cb, void());
};

/**
 * @brief Checks the correct behaviour of the ControllerStateMachine
 *
 * In this test the behaviour of the ControllerStateMachine with regards to its
 * callback handling is checked. Internals (such as concrete states) are ignored.
 */
class ControllerStateMachineTest : public ::testing::Test
{
protected:
  ControllerStateMachineTest();

  MonitoringFrameMsg generateMonitoringFrame() const;

protected:
  ControllerMock controller_;
  std::unique_ptr<ControllerStateMachine> state_machine_;
};

ControllerStateMachineTest::ControllerStateMachineTest()
{
  state_machine_.reset(
      new ControllerStateMachine(std::bind(&ControllerMock::monitoring_frame_cb, &controller_, std::placeholders::_1),
                                 std::bind(&ControllerMock::start_request_cb, &controller_),
                                 std::bind(&ControllerMock::stop_request_cb, &controller_),
                                 std::bind(&ControllerMock::started_cb, &controller_),
                                 std::bind(&ControllerMock::stopped_cb, &controller_)));
}

MonitoringFrameMsg ControllerStateMachineTest::generateMonitoringFrame() const
{
  const UDPFrameTestDataWithoutIntensities test_data;
  const psen_scan_v2::MaxSizeRawData raw_data = convertToMaxSizeRawData(test_data.hex_dump);
  return MonitoringFrameMsg::fromRawData(raw_data, raw_data.size());
}

TEST_F(ControllerStateMachineTest, testStartRequestCallbackIsCalled)
{
  EXPECT_CALL(controller_, start_request_cb()).Times(1);

  state_machine_->processStartRequestEvent();
}

TEST_F(ControllerStateMachineTest, testStartedCallbackIsCalled)
{
  EXPECT_CALL(controller_, start_request_cb()).Times(1);
  EXPECT_CALL(controller_, started_cb()).Times(1);

  state_machine_->processStartRequestEvent();
  state_machine_->processReplyReceivedEvent(ScannerReplyMsgType::Start);
}

MATCHER_P(MonitoringFrameEq, frame, "")
{
  return (arg.fromTheta() == frame.fromTheta() && arg.resolution() == frame.resolution() &&
          arg.scanCounter() == frame.scanCounter() && arg.measures() == arg.measures());
}

TEST_F(ControllerStateMachineTest, testMonitoringFrameCallbackIsCalled)
{
  const MonitoringFrameMsg frame{ generateMonitoringFrame() };

  EXPECT_CALL(controller_, start_request_cb()).Times(1);
  EXPECT_CALL(controller_, started_cb()).Times(1);
  EXPECT_CALL(controller_, monitoring_frame_cb(MonitoringFrameEq(frame))).Times(1);

  state_machine_->processStartRequestEvent();
  state_machine_->processReplyReceivedEvent(ScannerReplyMsgType::Start);
  state_machine_->processMonitoringFrameReceivedEvent(frame);
}

TEST_F(ControllerStateMachineTest, testStopRequestCallbackIsCalled)
{
  EXPECT_CALL(controller_, start_request_cb()).Times(1);
  EXPECT_CALL(controller_, started_cb()).Times(1);
  EXPECT_CALL(controller_, stop_request_cb()).Times(1);

  state_machine_->processStartRequestEvent();
  state_machine_->processReplyReceivedEvent(ScannerReplyMsgType::Start);
  state_machine_->processStopRequestEvent();
}

TEST_F(ControllerStateMachineTest, testStoppedCallbackIsCalled)
{
  EXPECT_CALL(controller_, start_request_cb()).Times(1);
  EXPECT_CALL(controller_, started_cb()).Times(1);
  EXPECT_CALL(controller_, stop_request_cb()).Times(1);
  EXPECT_CALL(controller_, stopped_cb()).Times(1);

  state_machine_->processStartRequestEvent();
  state_machine_->processReplyReceivedEvent(ScannerReplyMsgType::Start);
  state_machine_->processStopRequestEvent();
  state_machine_->processReplyReceivedEvent(ScannerReplyMsgType::Stop);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
