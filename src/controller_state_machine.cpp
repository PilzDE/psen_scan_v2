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

#include "psen_scan_v2/logging.h"
#include "psen_scan_v2/controller_state_machine.h"

namespace psen_scan_v2
{
ControllerStateMachine::ControllerStateMachine(const SendRequestCallback& start_request_cb,
                                               const SendRequestCallback& stop_request_cb,
                                               const StartedCallback& started_cb,
                                               const StoppedCallback& stopped_cb)
  : sm_(start_request_cb, stop_request_cb, started_cb, stopped_cb)
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.start();
}

ControllerStateMachine::~ControllerStateMachine()
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.send_start_request_callback_ = nullptr;
  sm_.stop();
}

void ControllerStateMachine::processStartRequestEvent()
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.process_event(udp_connection_state_machine::events::start_request());
}

// LCOV_EXCL_START
// TODO: Add again to coverage when function are actually used.
void ControllerStateMachine::processStartReplyReceivedEvent()
{
  PSENSCAN_INFO("Scanner", "Scanner started.");
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.process_event(udp_connection_state_machine::events::start_reply_received());
}

void ControllerStateMachine::processMonitoringFrameReceivedEvent()
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.process_event(udp_connection_state_machine::events::monitoring_frame_received());
}

void ControllerStateMachine::processStopRequestEvent()
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.process_event(udp_connection_state_machine::events::stop_request());
}

void ControllerStateMachine::processStopReplyReceivedEvent()
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.process_event(udp_connection_state_machine::events::stop_reply_received());
}
// LCOV_EXCL_STOP

}  // namespace psen_scan_v2
