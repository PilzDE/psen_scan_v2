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

inline ScannerProtocolDef::ScannerProtocolDef(StateMachineArgs* const args) : args_(args)
{
}

//+++++++++++++++++++++++++++++++++ States ++++++++++++++++++++++++++++++++++++

// clang-format off
#define DEFAULT_ON_ENTRY_IMPL(state_name)\
  template <class Event, class FSM>\
  void ScannerProtocolDef::state_name::on_entry(Event const&, FSM& fsm)\
  {\
    PSENSCAN_DEBUG("StateMachine", fmt::format("Entering state: {}", #state_name));\
  }\

#define DEFAULT_ON_EXIT_IMPL(state_name)\
  template <class Event, class FSM>\
  void ScannerProtocolDef::state_name::on_exit(Event const&, FSM& fsm)\
  {\
    PSENSCAN_DEBUG("StateMachine", fmt::format("Exiting state: {}", #state_name));\
  }

#define DEFAULT_STATE_IMPL(state_name)\
  DEFAULT_ON_ENTRY_IMPL(state_name)\
  DEFAULT_ON_EXIT_IMPL(state_name)
// clang-format on

DEFAULT_STATE_IMPL(WaitForStartReply)
DEFAULT_STATE_IMPL(WaitForStopReply)

DEFAULT_ON_ENTRY_IMPL(Idle)

template <class Event, class FSM>
void ScannerProtocolDef::Idle::on_exit(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", fmt::format("Exiting state: {}", "Idle"));
  fsm.args_->control_client_->startAsyncReceiving();
  fsm.args_->data_client_->startAsyncReceiving();
}

template <class Event, class FSM>
void ScannerProtocolDef::WaitForMonitoringFrame::on_entry(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", fmt::format("Entering state: {}", "WaitForMonitoringFrame"));
  fsm.args_->scanner_started_cb();
}

DEFAULT_ON_EXIT_IMPL(WaitForMonitoringFrame)

template <class Event, class FSM>
void ScannerProtocolDef::Stopped::on_entry(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", fmt::format("Entering state: {}", "Stopped"));
  fsm.args_->scanner_stopped_cb();
}

DEFAULT_ON_EXIT_IMPL(Stopped)

//+++++++++++++++++++++++++++++++++ Actions +++++++++++++++++++++++++++++++++++

static constexpr uint32_t DEFAULT_SEQ_NUMBER{ 0 };

template <class T>
inline void ScannerProtocolDef::sendStartRequest(const T& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: sendStartRequest");
  args_->control_client_->write(StartRequest(args_->config_, DEFAULT_SEQ_NUMBER).serialize());
}

inline void ScannerProtocolDef::sendStopRequest(const scanner_events::StopRequest& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: sendStopRequest");
  args_->data_client_->close();
  args_->control_client_->write(StopRequest().serialize());
}

inline void ScannerProtocolDef::handleMonitoringFrame(const scanner_events::RawMonitoringFrameReceived& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: handleMonitoringFrame");
  const MonitoringFrameMsg frame{ deserializeMonitoringFrame(event.data_, event.num_bytes_) };
  PSENSCAN_WARN_THROTTLE(
      1 /* sec */, "ScannerController", "The scanner reports an error: {}", frame.diagnosticMessages());
  args_->inform_user_about_laser_scan_cb(toLaserScan(frame));
}

//+++++++++++++++++++++++++++++++++ Guards ++++++++++++++++++++++++++++++++++++

inline bool ScannerProtocolDef::isStartReply(scanner_events::RawReplyReceived const& reply_event)
{
  const ScannerReplyMsg msg{ ScannerReplyMsg::deserialize(reply_event.data_) };
  return msg.type() == ScannerReplyMsgType::Start;
}

inline bool ScannerProtocolDef::isStopReply(scanner_events::RawReplyReceived const& reply_event)
{
  const ScannerReplyMsg msg{ ScannerReplyMsg::deserialize(reply_event.data_) };
  return msg.type() == ScannerReplyMsgType::Stop;
}

//++++++++++++++++++++ Special transitions ++++++++++++++++++++++++++++++++++++

template <class T>
static std::string classNameShort(const T& t)
{
  const auto full_name{ boost::core::demangle(typeid(t).name()) };
  return full_name.substr(full_name.rfind("::") + 2);
}

template <class FSM, class Event>
void ScannerProtocolDef::no_transition(Event const& event, FSM&, int state)
{
  PSENSCAN_WARN("StateMachine", "No transition in state {} for event {}.", state, classNameShort(event));
}

template <class FSM>
void ScannerProtocolDef::no_transition(const scanner_events::RawMonitoringFrameReceived&, FSM&, int state)
{
  PSENSCAN_WARN("StateMachine", "Received monitoring frame despite not waiting for it");
}
