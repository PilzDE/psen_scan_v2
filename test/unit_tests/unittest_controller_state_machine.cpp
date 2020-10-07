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

using namespace psen_scan_v2;

namespace psen_scan_v2_test
{
/**
 * @brief Checks the correct behaviour of the ControllerStateMachine
 *
 * In this test the behaviour of the ControllerStateMachine with regards to its
 * callback handling is checked. Internals (such as concrete states) are ignored.
 */
class ControllerStateMachineTest : public ::testing::Test
{
public:
  MOCK_METHOD0(send_start_request_callbackTest, void());
  MOCK_METHOD0(send_stop_request_callbackTest, void());
  MOCK_METHOD0(started_cb, void());
  MOCK_METHOD0(stopped_cb, void());
};

TEST_F(ControllerStateMachineTest, triggerSendRequestCallbackTest)
{
  EXPECT_CALL(*this, send_start_request_callbackTest()).Times(1);

  ControllerStateMachine sm(std::bind(&ControllerStateMachineTest::send_start_request_callbackTest, this),
                            std::bind(&ControllerStateMachineTest::send_stop_request_callbackTest, this),
                            std::bind(&ControllerStateMachineTest::started_cb, this),
                            std::bind(&ControllerStateMachineTest::stopped_cb, this));

  sm.processStartRequestEvent();
}

TEST_F(ControllerStateMachineTest, testThatStopRequestCallbackIsCalled)
{
  EXPECT_CALL(*this, send_start_request_callbackTest()).Times(1);
  EXPECT_CALL(*this, send_stop_request_callbackTest()).Times(1);

  ControllerStateMachine sm(std::bind(&ControllerStateMachineTest::send_start_request_callbackTest, this),
                            std::bind(&ControllerStateMachineTest::send_stop_request_callbackTest, this),
                            std::bind(&ControllerStateMachineTest::started_cb, this),
                            std::bind(&ControllerStateMachineTest::stopped_cb, this));

  sm.processStartRequestEvent();
  sm.processReplyReceivedEvent(ScannerReplyMsgType::Start);
  sm.processStopRequestEvent();
}

TEST_F(ControllerStateMachineTest, testThatStopReplyReceivedCallbackIsCalled)
{
  EXPECT_CALL(*this, send_start_request_callbackTest()).Times(1);
  EXPECT_CALL(*this, send_stop_request_callbackTest()).Times(1);
  EXPECT_CALL(*this, stopped_cb()).Times(1);

  ControllerStateMachine sm(std::bind(&ControllerStateMachineTest::send_start_request_callbackTest, this),
                            std::bind(&ControllerStateMachineTest::send_stop_request_callbackTest, this),
                            std::bind(&ControllerStateMachineTest::started_cb, this),
                            std::bind(&ControllerStateMachineTest::stopped_cb, this));

  sm.processStartRequestEvent();
  sm.processReplyReceivedEvent(ScannerReplyMsgType::Start);
  sm.processStopRequestEvent();
  sm.processReplyReceivedEvent(ScannerReplyMsgType::Stop);
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
