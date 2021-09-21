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
#ifndef PSEN_SCAN_V2_STANDALONE_SCANNER_PROTOCOL_DEF_H
#define PSEN_SCAN_V2_STANDALONE_SCANNER_PROTOCOL_DEF_H

#include <functional>
#include <string>
#include <memory>
#include <mutex>
#include <chrono>
#include <stdexcept>
#include <vector>

#define BOOST_MSM_CONSTRUCTOR_ARG_SIZE 10  // see https://www.boost.org/doc/libs/1_66_0/libs/msm/doc/HTML/ch03s05.html

// back-end
#include <boost/msm/back/state_machine.hpp>
// front-end
#include <boost/msm/front/state_machine_def.hpp>

#include <boost/msm/back/tools.hpp>
#include <boost/msm/back/metafunctions.hpp>

#include "psen_scan_v2_standalone/protocol_layer/scanner_events.h"

#include "psen_scan_v2_standalone/util/logging.h"
#include "psen_scan_v2_standalone/util/format_range.h"
#include "psen_scan_v2_standalone/util/ip_conversion.h"
#include "psen_scan_v2_standalone/communication_layer/udp_client.h"

#include "psen_scan_v2_standalone/laserscan.h"
#include "psen_scan_v2_standalone/data_conversion_layer/laserscan_conversions.h"

#include "psen_scan_v2_standalone/data_conversion_layer/start_request.h"
#include "psen_scan_v2_standalone/data_conversion_layer/stop_request_serialization.h"

#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/scanner_reply_serialization_deserialization.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_msg.h"
#include "psen_scan_v2_standalone/data_conversion_layer/monitoring_frame_deserialization.h"
#include "psen_scan_v2_standalone/protocol_layer/scan_buffer.h"
#include "psen_scan_v2_standalone/util/watchdog.h"

namespace psen_scan_v2_standalone
{
/**
 * @brief Contains all things needed to describe and implement the scanner protocol.
 */
namespace protocol_layer
{
namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace e = psen_scan_v2_standalone::protocol_layer::scanner_events;

// clang-format off
#define STATE(state_name)\
  class state_name : public msm::front::state<>\
{\
  public:\
  template <class Event, class FSM>\
  void on_entry(Event const&, FSM& fsm);\
  \
  template <class Event, class FSM>\
  void on_exit(Event const&, FSM& fsm);\
}
// clang-format on

static constexpr std::chrono::milliseconds WATCHDOG_TIMEOUT{ 1000 };
static constexpr uint32_t DEFAULT_NUM_MSG_PER_ROUND{ 6 };

using ScannerStartedCallback = std::function<void()>;
using ScannerStoppedCallback = std::function<void()>;
using TimeoutCallback = std::function<void()>;
using InformUserAboutLaserScanCallback = std::function<void(const LaserScan&)>;

/**
 * @brief Interface to create event timeout callbacks.
 *
 * Implementations of this Interface should create thread save timeout callbacks that
 * call an event every time a defined timeout has run out and restart themselves until deleted.
 *
 * @see util::Watchdog
 */
class IWatchdogFactory
{
public:
  virtual ~IWatchdogFactory() = default;

public:
  virtual std::unique_ptr<util::Watchdog> create(const util::Watchdog::Timeout& timeout,
                                                 const TimeoutCallback& timeout_callback) = 0;
};

/**
 * @brief Watchdog factory implementation for scanner interaction timeouts
 *
 * Implements the IWatchdogFactory to add behavior to handle specific cases,
 * where the interaction with the scanner hardware takes longer than expected.
 *
 * @see protocol_layer::IWatchdogFactory
 * @see util::Watchdog
 */
class WatchdogFactory : public IWatchdogFactory
{
public:
  WatchdogFactory() = default;
  std::unique_ptr<util::Watchdog> create(const util::Watchdog::Timeout& timeout,
                                         const TimeoutCallback& timeout_callback) override;
};

// front-end: define the FSM structure
/**
 * @brief Definition of the scanner protocol. It is initialized using the StateMachineArgs class.
 *
 * This class interacts with UdpClientImpl in order to perform its actions. These include sending a start request, a
 * stop request and handling incoming messages such as a start reply, a stop reply and a monitoring frame.
 *
 * Precisely, the StateMachineArgs::control_client_ is used for the starting-/stopping procedure and the
 * StateMachineArgs::data_client_ for receiving monitoring frames.
 *
 * It also checks for internal errors of incoming messages and handles timeouts of the above mentioned actions by
 * creating watchdogs via IWatchdogFactory.
 *
 * @see data_conversion_layer::start_request::Message
 * @see data_conversion_layer::stop_request
 * @see data_conversion_layer::scanner_reply::Message
 * @see data_conversion_layer::monitoring_frame::Message
 */
class ScannerProtocolDef : public msm::front::state_machine_def<ScannerProtocolDef>
{
public:
  ScannerProtocolDef(const ScannerConfiguration config,
                     const communication_layer::NewMessageCallback& control_msg_callback,
                     const communication_layer::ErrorCallback& control_error_callback,
                     const communication_layer::NewMessageCallback& data_msg_callback,
                     const communication_layer::ErrorCallback& data_error_callback,
                     const ScannerStartedCallback& scanner_started_callback,
                     const ScannerStoppedCallback& scanner_stopped_callback,
                     const InformUserAboutLaserScanCallback& laser_scan_callback,
                     const TimeoutCallback& start_timeout_callback,
                     const TimeoutCallback& monitoring_frame_timeout_callback);

public:  // States
  STATE(Idle);
  STATE(WaitForStartReply);
  STATE(WaitForMonitoringFrame);
  STATE(WaitForStopReply);
  STATE(Stopped);

public:  // Action methods
  template <class T>
  void sendStartRequest(const T& event);
  void handleStartRequestTimeout(const scanner_events::StartTimeout& event);
  template <class T>
  void sendStopRequest(const T& event);
  void handleMonitoringFrame(const scanner_events::RawMonitoringFrameReceived& event);
  void handleMonitoringFrameTimeout(const scanner_events::MonitoringFrameTimeout& event);

public:  // Guards
  bool isStartReply(scanner_events::RawReplyReceived const& reply_event);
  bool isStopReply(scanner_events::RawReplyReceived const& reply_event);

public:  // Replaces the default exception/no-transition responses
  template <class FSM, class Event>
  void exception_caught(Event const& event, FSM& fsm, std::exception& exception);

  template <class FSM, class Event>
  void no_transition(Event const& event, FSM&, int state);

  template <class FSM>
  void no_transition(const scanner_events::RawMonitoringFrameReceived&, FSM&, int state);

public:  // Definition of state machine via table
  typedef Idle initial_state;
  typedef ScannerProtocolDef m;

  // clang-format off
  /**
   * @brief Table describing the state machine which is specified in the scanner protocol.
   */
  struct transition_table : mpl::vector<
      //    Start                         Event                         Next                        Action                        Guard
      //  +------------------------------+----------------------------+---------------------------+------------------------------+-----------------------------+
      a_row  < Idle,                      e::StartRequest,              WaitForStartReply,          &m::sendStartRequest                                      >,
      a_row  < Idle,                      e::StopRequest,               WaitForStopReply,           &m::sendStopRequest                                       >,
      g_row  < WaitForStartReply,         e::RawReplyReceived,          WaitForMonitoringFrame,                                   &m::isStartReply            >,
      a_irow < WaitForStartReply,         e::StartTimeout,                                          &m::handleStartRequestTimeout                             >,
      a_irow < WaitForMonitoringFrame,    e::RawMonitoringFrameReceived,                            &m::handleMonitoringFrame                                 >,
      a_irow < WaitForMonitoringFrame,    e::MonitoringFrameTimeout,                                &m::handleMonitoringFrameTimeout                          >,
      a_row  < WaitForStartReply,         e::StopRequest,               WaitForStopReply,           &m::sendStopRequest                                       >,
      a_row  < WaitForMonitoringFrame,    e::StopRequest,               WaitForStopReply,           &m::sendStopRequest                                       >,
      _irow  < WaitForStopReply,          e::RawMonitoringFrameReceived                                                                                       >,
      g_row  < WaitForStopReply,          e::RawReplyReceived,          Stopped,                                                  &m::isStopReply             >,
      _irow  < Stopped,                   e::RawMonitoringFrameReceived                                                                                       >
      //  +------------------------------+----------------------------+--------------------------+--------------------------------+-----------------------------+
      > {};
  // clang-format on

private:
  // LCOV_EXCL_START
  /**
   * @brief Exception thrown when something goes wrong with the scanner reply.
   *
   * For example an unexpected code in reply, request refused by device or unknown operation result code.
   */
  class InternalScannerReplyError : public std::runtime_error
  {
  public:
    InternalScannerReplyError(const std::string& error_msg);
  };
  // LCOV_EXCL_STOP
  void checkForInternalErrors(const data_conversion_layer::scanner_reply::Message& msg);

  void checkForDiagnosticErrors(const data_conversion_layer::monitoring_frame::Message& msg);
  void informUserAboutTheScanData(const data_conversion_layer::monitoring_frame::MessageStamped& stamped_msg);
  void
  sendMessageWithMeasurements(const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msg);
  bool
  framesContainMeasurements(const std::vector<data_conversion_layer::monitoring_frame::MessageStamped>& stamped_msg);

private:
  ScannerConfiguration config_;

  std::unique_ptr<util::Watchdog> start_reply_watchdog_{};

  std::unique_ptr<util::Watchdog> monitoring_frame_watchdog_{};
  ScanBuffer scan_buffer_{ DEFAULT_NUM_MSG_PER_ROUND };

  // Udp Clients
  communication_layer::UdpClientImpl control_client_;
  communication_layer::UdpClientImpl data_client_;

  // Callbacks
  const ScannerStartedCallback scanner_started_callback_;
  const ScannerStoppedCallback scanner_stopped_callback_;
  const InformUserAboutLaserScanCallback inform_user_about_laser_scan_callback_;

  // Timeout Handler
  const std::function<void()> start_timeout_callback_;
  const std::function<void()> monitoring_frame_timeout_callback_;

  // Factories
  WatchdogFactory watchdog_factory_{};
};

// Pick a back-end
/**
 * @brief State machine handling all events according to the scanner protocol and error handling specification.
 *
 * @see protocol_layer::ScannerProtocolDef
 */
using ScannerStateMachine = msm::back::state_machine<ScannerProtocolDef>;

}  // namespace protocol_layer
}  // namespace psen_scan_v2_standalone

#include "psen_scan_v2_standalone/protocol_layer/scanner_state_machine_def.h"

#endif  // PSEN_SCAN_V2_STANDALONE_SCANNER_PROTOCOL_DEF_H
