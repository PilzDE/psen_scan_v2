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

#ifndef PSEN_SCAN_V2_TEST_STATE_MACHINE_CONTROLLER_MOCK_H
#define PSEN_SCAN_V2_TEST_STATE_MACHINE_CONTROLLER_MOCK_H

#include <gmock/gmock.h>

#include "psen_scan_v2/controller_state_machine.h"
#include "psen_scan_v2/function_pointers.h"

namespace psen_scan_v2_test
{
class ControllerStateMachineMock
{
public:
  ControllerStateMachineMock(const psen_scan_v2::SendRequestCallback& start_request_cb,
                             const psen_scan_v2::SendRequestCallback& stop_request_cb,
                             const psen_scan_v2::StartedCallback& started_cb,
                             const psen_scan_v2::StoppedCallback& stopped_cb)
    : started_cb_(started_cb), stopped_cb_(stopped_cb){};

public:
  MOCK_METHOD0(processStartRequestEvent, void());
  MOCK_METHOD0(processStartReplyReceivedEvent, void());
  MOCK_METHOD0(processMonitoringFrameReceivedEvent, void());
  MOCK_METHOD0(processStopRequestEvent, void());
  MOCK_METHOD0(processStopReplyReceivedEvent, void());

  const psen_scan_v2::StoppedCallback started_cb_;
  const psen_scan_v2::StoppedCallback stopped_cb_;
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_STATE_MACHINE_CONTROLLER_MOCK_H
