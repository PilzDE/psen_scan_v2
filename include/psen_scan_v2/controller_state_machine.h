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

#ifndef PSEN_SCAN_V2_CONTROLLER_STATE_MACHINE_H
#define PSEN_SCAN_V2_CONTROLLER_STATE_MACHINE_H

#include <mutex>
#include "psen_scan_v2/udp_connection_state_machine.h"

namespace psen_scan_v2
{
/**
 * @brief State machine reprensenting the communication protocol of the scanner.
 */
class ControllerStateMachine
{
public:
  explicit ControllerStateMachine(const SendRequestCallback& start_request_cb,
                                  const SendRequestCallback& stop_request_cb,
                                  const StoppedCallback& stopped_cb);
  virtual ~ControllerStateMachine();

  void processStartRequestEvent();
  void processStartReplyReceivedEvent();
  void processMonitoringFrameReceivedEvent();
  void processStopRequestEvent();
  void processStopReplyReceivedEvent();

private:
  template <typename T>
  void processEvent();

private:
  udp_connection_state_machine sm_;
  std::mutex sm_access_mutex_;
};

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_CONTROLLER_STATE_MACHINE_H
