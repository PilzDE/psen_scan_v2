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

namespace psen_scan_v2_standalone
{
namespace scanner_protocol
{
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
void ScannerProtocolDef::WaitForStartReply::on_entry(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", fmt::format("Entering state: {}", "WaitForStartReply"));
  // Start watchdog...
  fsm.start_reply_watchdog_ = fsm.args_->watchdog_factory_->create(WATCHDOG_TIMEOUT, "StartReplyTimeout");
}

template <class Event, class FSM>
void ScannerProtocolDef::WaitForStartReply::on_exit(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", fmt::format("Exiting state: {}", "WaitForStartReply"));
  // Stops the watchdog by resetting the pointer
  fsm.start_reply_watchdog_.reset();
}

template <class Event, class FSM>
void ScannerProtocolDef::WaitForMonitoringFrame::on_entry(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", fmt::format("Entering state: {}", "WaitForMonitoringFrame"));
  fsm.complete_scan_validator_.reset();
  // Start watchdog...
  fsm.monitoring_frame_watchdog_ = fsm.args_->watchdog_factory_->create(WATCHDOG_TIMEOUT, "MonitoringFrameTimeout");
  fsm.args_->scanner_started_cb();
}

template <class Event, class FSM>
void ScannerProtocolDef::WaitForMonitoringFrame::on_exit(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", fmt::format("Exiting state: {}", "WaitForMonitoringFrame"));
  // Stops the watchdog by resetting the pointer
  fsm.monitoring_frame_watchdog_.reset();
}

template <class Event, class FSM>
void ScannerProtocolDef::Stopped::on_entry(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", fmt::format("Entering state: {}", "Stopped"));
  fsm.args_->scanner_stopped_cb();
}

DEFAULT_ON_EXIT_IMPL(Stopped)

//+++++++++++++++++++++++++++++++++ Actions +++++++++++++++++++++++++++++++++++

template <class T>
inline void ScannerProtocolDef::sendStartRequest(const T& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: sendStartRequest");
  args_->control_client_->write(serialize(start_request::Message(args_->config_)));
}

inline void ScannerProtocolDef::handleStartRequestTimeout(const scanner_events::StartTimeout& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: handleStartRequestTimeout");
  PSENSCAN_ERROR("StateMachine",
                 "Timeout while waiting for the scanner to start! Retrying... "
                 "(Please check the ethernet connection or contact PILZ support if the error persists.)");
  sendStartRequest(event);
}

inline void ScannerProtocolDef::sendStopRequest(const scanner_events::StopRequest& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: sendStopRequest");
  args_->data_client_->close();
  args_->control_client_->write(stop_request::serialize());
}

inline void ScannerProtocolDef::printUserMsgFor(const ScanValidatorResult& res)
{
  using Result = monitoring_frame::ScanValidator::Result;
  if (!res || res.value() == Result::valid)
  {
    return;
  }

  if (res.value() == Result::undersaturated)
  {
    PSENSCAN_WARN("StateMachine",
                  "Detected dropped MonitoringFrame."
                  " (Please check the ethernet connection or contact PILZ support if the error persists.)");
    return;
  }

  PSENSCAN_WARN("StateMachine", "Unexpected: Too many MonitoringFrames for one scan round received.");
}

inline void ScannerProtocolDef::handleMonitoringFrame(const scanner_events::RawMonitoringFrameReceived& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: handleMonitoringFrame");
  monitoring_frame_watchdog_->reset();
  const monitoring_frame::Message frame{ monitoring_frame::deserialize(event.data_, event.num_bytes_) };
  PSENSCAN_WARN_THROTTLE(
      1 /* sec */, "StateMachine", "The scanner reports an error: {}", formatRange(frame.diagnosticMessages()));

  printUserMsgFor(complete_scan_validator_.validate(frame, DEFAULT_NUM_MSG_PER_ROUND));
  args_->inform_user_about_laser_scan_cb(toLaserScan(frame));
}

inline void ScannerProtocolDef::handleMonitoringFrameTimeout(const scanner_events::MonitoringFrameTimeout& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: handleMonitoringFrameTimeout");

  PSENSCAN_WARN("StateMachine",
                "Timeout while waiting for MonitoringFrame message."
                " (Please check the ethernet connection or contact PILZ support if the error persists.)");
}

//+++++++++++++++++++++++++++++++++ Guards ++++++++++++++++++++++++++++++++++++

inline bool ScannerProtocolDef::isStartReply(scanner_events::RawReplyReceived const& reply_event)
{
  const scanner_reply::Message msg{ scanner_reply::deserialize(reply_event.data_) };
  return msg.type() == scanner_reply::Message::Type::start;
}

inline bool ScannerProtocolDef::isStopReply(scanner_events::RawReplyReceived const& reply_event)
{
  const scanner_reply::Message msg{ scanner_reply::deserialize(reply_event.data_) };
  return msg.type() == scanner_reply::Message::Type::stop;
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

}  // namespace scanner_protocol
}  // namespace psen_scan_v2_standalone
