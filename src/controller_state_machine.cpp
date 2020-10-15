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
#include "psen_scan_v2/scanner_reply_msg.h"

namespace psen_scan_v2
{
ControllerStateMachine::ControllerStateMachine(const MonitoringFrameCallback& monitoring_frame_cb,
                                               const SendRequestCallback& start_request_cb,
                                               const SendRequestCallback& stop_request_cb,
                                               const StartedCallback& started_cb,
                                               const StoppedCallback& stopped_cb)
  : sm_(monitoring_frame_cb, start_request_cb, stop_request_cb, started_cb, stopped_cb)
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

void ControllerStateMachine::processStartReplyTimeoutEvent()
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.process_event(udp_connection_state_machine::events::start_reply_timeout());
}

// LCOV_EXCL_START
// TODO: Add again to coverage when function are actually used.
void ControllerStateMachine::processReplyReceivedEvent(ScannerReplyMsgType type)
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.process_event(udp_connection_state_machine::events::reply_received(type));
}

void ControllerStateMachine::processMonitoringFrameReceivedEvent(const MonitoringFrameMsg& frame)
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.process_event(udp_connection_state_machine::events::monitoring_frame_received(frame));
}

void ControllerStateMachine::processStopRequestEvent()
{
  const std::lock_guard<std::mutex> lock(sm_access_mutex_);
  sm_.process_event(udp_connection_state_machine::events::stop_request());
}
// LCOV_EXCL_STOP

}  // namespace psen_scan_v2
