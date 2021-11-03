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

#include "psen_scan_v2_standalone/data_conversion_layer/start_request_serialization.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/communication_layer/udp_client.h"
namespace psen_scan_v2_standalone
{
namespace protocol_layer
{
inline ScannerProtocolDef::ScannerProtocolDef(const ScannerConfiguration config,
                                              const communication_layer::NewMessageCallback& control_msg_callback,
                                              const communication_layer::ErrorCallback& control_error_callback,
                                              const communication_layer::ErrorCallback& start_error_callback,
                                              const communication_layer::NewMessageCallback& data_msg_callback,
                                              const communication_layer::ErrorCallback& data_error_callback,
                                              const ScannerStartedCallback& scanner_started_callback,
                                              const ScannerStoppedCallback& scanner_stopped_callback,
                                              const InformUserAboutLaserScanCallback& laser_scan_callback,
                                              const TimeoutCallback& start_timeout_callback,
                                              const TimeoutCallback& monitoring_frame_timeout_callback)
  : config_(config)
  , control_client_(control_msg_callback,
                    control_error_callback,
                    config_.hostUDPPortControl(),  // LCOV_EXCL_LINE Lcov bug?
                    config_.clientIp(),
                    config_.scannerControlPort())
  , data_client_(data_msg_callback,
                 data_error_callback,
                 config_.hostUDPPortData(),  // LCOV_EXCL_LINE Lcov bug?
                 config_.clientIp(),
                 config_.scannerDataPort())
  , scanner_started_callback_(scanner_started_callback)
  , scanner_stopped_callback_(scanner_stopped_callback)
  , start_error_callback_(start_error_callback)
  , inform_user_about_laser_scan_callback_(laser_scan_callback)
  , start_timeout_callback_(start_timeout_callback)
  , monitoring_frame_timeout_callback_(monitoring_frame_timeout_callback)
{
}

//+++++++++++++++++++++++++++++++++ States ++++++++++++++++++++++++++++++++++++

// clang-format off
#define DEFAULT_ON_ENTRY_IMPL(state_name)\
  template <class Event, class FSM>\
  void ScannerProtocolDef::state_name::on_entry(Event const&, FSM& fsm)\
  {\
    PSENSCAN_DEBUG("StateMachine", "Entering state: " #state_name);\
  }\

#define DEFAULT_ON_EXIT_IMPL(state_name)\
  template <class Event, class FSM>\
  void ScannerProtocolDef::state_name::on_exit(Event const&, FSM& fsm)\
  {\
    PSENSCAN_DEBUG("StateMachine", "Exiting state: " #state_name);\
  }

#define DEFAULT_STATE_IMPL(state_name)\
  DEFAULT_ON_ENTRY_IMPL(state_name)\
  DEFAULT_ON_EXIT_IMPL(state_name)
// clang-format on

DEFAULT_STATE_IMPL(WaitForStopReply)

DEFAULT_ON_ENTRY_IMPL(Idle)

// \cond Ignore "was not declared or defined" warnings from doxygen
template <class Event, class FSM>
void ScannerProtocolDef::Idle::on_exit(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", "Exiting state: Idle");
  fsm.control_client_.startAsyncReceiving();
  fsm.data_client_.startAsyncReceiving();
}

template <class Event, class FSM>
void ScannerProtocolDef::WaitForStartReply::on_entry(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", "Entering state: WaitForStartReply");
  // Start watchdog...
  fsm.start_reply_watchdog_ = fsm.watchdog_factory_.create(WATCHDOG_TIMEOUT, fsm.start_timeout_callback_);
}

template <class Event, class FSM>
void ScannerProtocolDef::WaitForStartReply::on_exit(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", "Exiting state: WaitForStartReply");
  // Stops the watchdog by resetting the pointer
  fsm.start_reply_watchdog_.reset();
}

template <class Event, class FSM>
void ScannerProtocolDef::WaitForMonitoringFrame::on_entry(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", "Entering state: WaitForMonitoringFrame");
  fsm.scan_buffer_.reset();
  // Start watchdog...
  fsm.monitoring_frame_watchdog_ =
      fsm.watchdog_factory_.create(WATCHDOG_TIMEOUT, fsm.monitoring_frame_timeout_callback_);
}

template <class Event, class FSM>
void ScannerProtocolDef::WaitForMonitoringFrame::on_exit(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", "Exiting state: WaitForMonitoringFrame");
  // Stops the watchdog by resetting the pointer
  fsm.monitoring_frame_watchdog_.reset();
}

template <class Event, class FSM>
void ScannerProtocolDef::Stopped::on_entry(Event const&, FSM& fsm)
{
  PSENSCAN_DEBUG("StateMachine", "Entering state: Stopped");
}

DEFAULT_ON_EXIT_IMPL(Stopped)

DEFAULT_ON_ENTRY_IMPL(Error)
DEFAULT_ON_EXIT_IMPL(Error)

// \endcond
//+++++++++++++++++++++++++++++++++ Actions +++++++++++++++++++++++++++++++++++

template <class T>
inline void ScannerProtocolDef::sendStartRequest(const T& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: sendStartRequest");

  if (!config_.hostIp())
  {
    auto host_ip{ control_client_.getHostIp() };
    config_.setHostIp(host_ip.to_ulong());
    PSENSCAN_INFO("StateMachine", "No host ip set! Using local ip: {}", host_ip.to_string());
  }
  control_client_.write(
      data_conversion_layer::start_request::serialize(data_conversion_layer::start_request::Message(config_)));
}

inline void ScannerProtocolDef::handleStartRequestTimeout(const scanner_events::StartTimeout& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: handleStartRequestTimeout");
  PSENSCAN_ERROR("StateMachine",
                 "Timeout while waiting for the scanner to start! Retrying... "
                 "(Please check the ethernet connection or contact PILZ support if the error persists.)");
  sendStartRequest(event);
}

template <class T>
inline void ScannerProtocolDef::sendStopRequest(const T& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: sendStopRequest");
  data_client_.stop();
  control_client_.write(data_conversion_layer::stop_request::serialize());
}

inline void ScannerProtocolDef::handleMonitoringFrame(const scanner_events::RawMonitoringFrameReceived& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: handleMonitoringFrame");
  monitoring_frame_watchdog_->reset();

  try
  {
    const data_conversion_layer::monitoring_frame::Message msg{ data_conversion_layer::monitoring_frame::deserialize(
        *(event.data_), event.num_bytes_) };
    checkForDiagnosticErrors(msg);
    checkForChangedActiveZoneset(msg);
    const data_conversion_layer::monitoring_frame::MessageStamped stamped_msg{ msg, event.timestamp_ };
    informUserAboutTheScanData(stamped_msg);
  }
  // LCOV_EXCL_START
  catch (const data_conversion_layer::monitoring_frame::ScanCounterMissing& e)
  {
    PSENSCAN_ERROR("StateMachine", e.what());
  }
  // LCOV_EXCL_STOP
}

inline void ScannerProtocolDef::notifyUserAboutStart(scanner_events::RawReplyReceived const& reply_event)
{
  scanner_started_callback_();
}

inline void ScannerProtocolDef::notifyUserAboutStop(scanner_events::RawReplyReceived const& reply_event)
{
  scanner_stopped_callback_();
}

inline void ScannerProtocolDef::notifyUserAboutUnknownStartReply(scanner_events::RawReplyReceived const& reply_event)
{
  const data_conversion_layer::scanner_reply::Message msg{ data_conversion_layer::scanner_reply::deserialize(
      *(reply_event.data_)) };
  start_error_callback_(
      fmt::format("Unknown result code {:#04x} in start reply.", static_cast<uint32_t>(msg.result())));
}

inline void ScannerProtocolDef::notifyUserAboutRefusedStartReply(scanner_events::RawReplyReceived const& reply_event)
{
  start_error_callback_("Request refused by device.");
}

inline void ScannerProtocolDef::checkForDiagnosticErrors(const data_conversion_layer::monitoring_frame::Message& msg)
{
  if (!msg.diagnosticMessages().empty())
  {
    PSENSCAN_WARN_THROTTLE(
        1 /* sec */, "StateMachine", "The scanner reports an error: {}", util::formatRange(msg.diagnosticMessages()));
  }
}

inline void
ScannerProtocolDef::checkForChangedActiveZoneset(const data_conversion_layer::monitoring_frame::Message& msg)
{
  if (!zoneset_reference_msg_.is_initialized() || (msg.scanCounter() >= zoneset_reference_msg_->scanCounter() &&
                                                   msg.activeZoneset() != zoneset_reference_msg_->activeZoneset()))
  {
    PSENSCAN_INFO("Scanner", "The scanner switched to active zoneset {}", msg.activeZoneset());
    zoneset_reference_msg_ = msg;
  }
}

inline void ScannerProtocolDef::informUserAboutTheScanData(
    const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg)
{
  try
  {
    scan_buffer_.add(stamped_msg);
    if (!config_.fragmentedScansEnabled() && scan_buffer_.isRoundComplete())
    {
      sendMessageWithMeasurements(scan_buffer_.getMsgs());
    }
  }
  catch (const ScanRoundError& ex)
  {
    PSENSCAN_WARN("ScanBuffer", ex.what());
  }
  if (config_.fragmentedScansEnabled())  // Send the scan fragment in any case.
  {
    sendMessageWithMeasurements({ stamped_msg });
  }
}

inline void ScannerProtocolDef::sendMessageWithMeasurements(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs)
{
  if (framesContainMeasurements(stamped_msgs))
  {
    try
    {
      inform_user_about_laser_scan_callback_(data_conversion_layer::LaserScanConverter::toLaserScan(stamped_msgs));
    }
    // LCOV_EXCL_START
    catch (const data_conversion_layer::ScannerProtocolViolationError& ex)
    {
      PSENSCAN_ERROR("StateMachine", ex.what());
    }
    // LCOV_EXCL_STOP
  }
}

inline bool ScannerProtocolDef::framesContainMeasurements(
    const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msgs)
{
  if (std::all_of(stamped_msgs.begin(), stamped_msgs.end(), [](const auto& stamped_msg) {
        return stamped_msg.msg_.measurements().empty();
      }))
  {
    PSENSCAN_DEBUG("StateMachine", "No measurement data in current monitoring frame(s), skipping laser scan callback.");
    return false;
  }
  return true;
}

inline void ScannerProtocolDef::handleMonitoringFrameTimeout(const scanner_events::MonitoringFrameTimeout& event)
{
  PSENSCAN_DEBUG("StateMachine", "Action: handleMonitoringFrameTimeout");

  PSENSCAN_WARN("StateMachine",
                "Timeout while waiting for MonitoringFrame message."
                " (Please check the ethernet connection or contact PILZ support if the error persists.)");
}

//+++++++++++++++++++++++++++++++++ Guards ++++++++++++++++++++++++++++++++++++

// LCOV_EXCL_START
inline ScannerProtocolDef::InternalScannerReplyError::InternalScannerReplyError(const std::string& error_msg)
  : std::runtime_error(error_msg)
{
}
// LCOV_EXCL_STOP

inline void ScannerProtocolDef::checkForInternalErrors(const data_conversion_layer::scanner_reply::Message& msg)
{
  // LCOV_EXCL_START
  if (msg.type() == data_conversion_layer::scanner_reply::Message::Type::unknown)
  {
    throw InternalScannerReplyError("Unexpected code in reply");
  }
  if (msg.result() != data_conversion_layer::scanner_reply::Message::OperationResult::accepted)
  {
    if (msg.result() == data_conversion_layer::scanner_reply::Message::OperationResult::refused)
    {
      throw InternalScannerReplyError("Request refused by device.");
    }
    else
    {
      throw InternalScannerReplyError("Unknown operation result code.");
    }
  }
  // LCOV_EXCL_STOP
}

inline bool ScannerProtocolDef::isAcceptedStartReply(scanner_events::RawReplyReceived const& reply_event)
{
  const data_conversion_layer::scanner_reply::Message msg{ data_conversion_layer::scanner_reply::deserialize(
      *(reply_event.data_)) };
  return isStartReply(msg) && isAcceptedReply(msg);
}

inline bool ScannerProtocolDef::isUnknownStartReply(scanner_events::RawReplyReceived const& reply_event)
{
  const data_conversion_layer::scanner_reply::Message msg{ data_conversion_layer::scanner_reply::deserialize(
      *(reply_event.data_)) };
  return isStartReply(msg) && isUnknownReply(msg);
}

inline bool ScannerProtocolDef::isRefusedStartReply(scanner_events::RawReplyReceived const& reply_event)
{
  const data_conversion_layer::scanner_reply::Message msg{ data_conversion_layer::scanner_reply::deserialize(
      *(reply_event.data_)) };
  return isStartReply(msg) && isRefusedReply(msg);
}

inline bool ScannerProtocolDef::isAcceptedReply(data_conversion_layer::scanner_reply::Message const& msg)
{
  return msg.result() == data_conversion_layer::scanner_reply::Message::OperationResult::accepted;
}

inline bool ScannerProtocolDef::isUnknownReply(data_conversion_layer::scanner_reply::Message const& msg)
{
  return msg.result() == data_conversion_layer::scanner_reply::Message::OperationResult::unknown;
}

inline bool ScannerProtocolDef::isRefusedReply(data_conversion_layer::scanner_reply::Message const& msg)
{
  return msg.result() == data_conversion_layer::scanner_reply::Message::OperationResult::refused;
}

inline bool ScannerProtocolDef::isStartReply(data_conversion_layer::scanner_reply::Message const& msg)
{
  return msg.type() == data_conversion_layer::scanner_reply::Message::Type::start;
}

inline bool ScannerProtocolDef::isStopReply(scanner_events::RawReplyReceived const& reply_event)
{
  const data_conversion_layer::scanner_reply::Message msg{ data_conversion_layer::scanner_reply::deserialize(
      *(reply_event.data_)) };
  checkForInternalErrors(msg);
  return msg.type() == data_conversion_layer::scanner_reply::Message::Type::stop;
}

//++++++++++++++++++++ Special transitions ++++++++++++++++++++++++++++++++++++

template <class FSM>
static std::string getStateName(const int& state_id)
{
  using recursive_transition_table = typename boost::msm::back::recursive_get_transition_table<FSM>::type;
  using states = typename boost::msm::back::generate_state_set<recursive_transition_table>::type;

  std::string mangle_state_name;
  boost::mpl::for_each<states, boost::msm::wrap<boost::mpl::placeholders::_1> >(
      boost::msm::back::get_state_name<recursive_transition_table>(mangle_state_name, state_id));
  const auto full_name{ boost::core::demangle(mangle_state_name.c_str()) };
  return full_name.substr(full_name.rfind("::") + 2);
}

template <class T>
static std::string classNameShort(const T& t)
{
  const auto full_name{ boost::core::demangle(typeid(t).name()) };
  return full_name.substr(full_name.rfind("::") + 2);
}

// LCOV_EXCL_START
template <class FSM, class Event>
void ScannerProtocolDef::exception_caught(Event const& event, FSM& fsm, std::exception& exception)
{
  PSENSCAN_ERROR("StateMachine", "Received error \"{}\". Shutting down now.", exception.what());
  sendStopRequest(event);
  throw exception;
}
// LCOV_EXCL_STOP

template <class FSM, class Event>
void ScannerProtocolDef::no_transition(Event const& event, FSM&, int state)
{
  PSENSCAN_WARN("StateMachine",
                "No transition in state \"{}\" for event \"{}\".",
                getStateName<FSM>(state),
                classNameShort(event));
}

template <class FSM>
void ScannerProtocolDef::no_transition(const scanner_events::RawMonitoringFrameReceived&, FSM&, int state)
{
  PSENSCAN_WARN("StateMachine", "Received monitoring frame despite not waiting for it");
}

}  // namespace protocol_layer
}  // namespace psen_scan_v2_standalone
