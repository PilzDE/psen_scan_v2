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
#ifndef PSEN_SCAN_V2_SCANNER_PROTOCOL_DEF_H
#define PSEN_SCAN_V2_SCANNER_PROTOCOL_DEF_H

#include <functional>
#include <string>
#include <memory>
#include <mutex>
#include <chrono>

// back-end
#include <boost/msm/back/state_machine.hpp>
// front-end
#include <boost/msm/front/state_machine_def.hpp>

#include <boost/core/demangle.hpp>

#include "psen_scan_v2/scanner_events.h"

#include "psen_scan_v2/logging.h"
#include "psen_scan_v2/format_range.h"
#include "psen_scan_v2/udp_client.h"

#include "psen_scan_v2/laserscan.h"
#include "psen_scan_v2/laserscan_conversions.h"

#include "psen_scan_v2/start_request.h"
#include "psen_scan_v2/stop_request_serialization.h"

#include "psen_scan_v2/scanner_reply_msg.h"
#include "psen_scan_v2/scanner_reply_serialization_deserialization.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/monitoring_frame_deserialization.h"
#include "psen_scan_v2/complete_scan_validator.h"
#include "psen_scan_v2/watchdog.h"

namespace psen_scan_v2
{
/**
 * @brief Contains all things needed to describe and implement the scanner protocol.
 */
namespace scanner_protocol
{
namespace msm = boost::msm;
namespace mpl = boost::mpl;

namespace e = psen_scan_v2::scanner_protocol::scanner_events;

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

using ScannerStartedCB = std::function<void()>;
using ScannerStoppedCB = std::function<void()>;
using InformUserAboutLaserScanCB = std::function<void(const LaserScan&)>;

/**
 * @brief Interface to create event timeout handlers.
 *
 * Implementations of this Interface should create thread save timeout handlers that
 * call an event every time a defined timeout has run out and restart themselves until deleted.
 *
 * @see Watchdog
 */
class IWatchdogFactory
{
public:
  virtual ~IWatchdogFactory() = default;

public:
  virtual std::unique_ptr<Watchdog> create(const Watchdog::Timeout& timeout, const std::string& event_type) = 0;
};

/**
 * @brief Helper class used to easily transfer data from the higher level ScannerV2 class
 * to the ScannerProtocolDef class during construction of the ScannerStateMachine.
 */
struct StateMachineArgs
{
  StateMachineArgs(const ScannerConfiguration& scanner_config,
                   std::unique_ptr<UdpClientImpl> control_client,
                   std::unique_ptr<UdpClientImpl> data_client,
                   const ScannerStartedCB& started_cb,
                   const ScannerStoppedCB& stopped_cb,
                   const InformUserAboutLaserScanCB& laser_scan_cb,
                   std::unique_ptr<IWatchdogFactory> watchdog_factory)
    : config_(scanner_config)
    , scanner_started_cb(started_cb)
    , scanner_stopped_cb(stopped_cb)
    , inform_user_about_laser_scan_cb(laser_scan_cb)
    , watchdog_factory_(std::move(watchdog_factory))
    , control_client_(std::move(control_client))
    , data_client_(std::move(data_client))
  {
  }

  const ScannerConfiguration config_;

  // Callbacks
  const ScannerStartedCB scanner_started_cb{};
  const ScannerStoppedCB scanner_stopped_cb{};
  const InformUserAboutLaserScanCB inform_user_about_laser_scan_cb{};

  // Factories
  std::unique_ptr<IWatchdogFactory> watchdog_factory_{};

  // UDP clients
  // Note: The clients must be declared last, to ensure that they are desroyed first.
  // If they are not declared last, segmentation default might occur!
  std::unique_ptr<UdpClientImpl> control_client_{};
  std::unique_ptr<UdpClientImpl> data_client_{};
};

// front-end: define the FSM structure
/**
 * @brief Definition of the scanner protocol.
 */
class ScannerProtocolDef : public msm::front::state_machine_def<ScannerProtocolDef>
{
public:
  ScannerProtocolDef(StateMachineArgs* const args);

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
  void sendStopRequest(const scanner_events::StopRequest& event);
  void handleMonitoringFrame(const scanner_events::RawMonitoringFrameReceived& event);
  void handleMonitoringFrameTimeout(const scanner_events::MonitoringFrameTimeout& event);

public:  // Guards
  bool isStartReply(scanner_events::RawReplyReceived const& reply_event);
  bool isStopReply(scanner_events::RawReplyReceived const& reply_event);

public:  // Replaces the default no-transition responses
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
      g_row  < WaitForStopReply,          e::RawReplyReceived,          Stopped,                                                  &m::isStopReply             >
      //  +------------------------------+----------------------------+--------------------------+--------------------------------+-----------------------------+
      > {};
  // clang-format on

private:
  using ScanValidatorResult = monitoring_frame::ScanValidator::OptionalResult;
  void printUserMsgFor(const ScanValidatorResult& validation_result);

private:
  const std::unique_ptr<StateMachineArgs> args_;

  std::unique_ptr<Watchdog> start_reply_watchdog_{};

  std::unique_ptr<Watchdog> monitoring_frame_watchdog_{};
  monitoring_frame::ScanValidator complete_scan_validator_;
};

#include "psen_scan_v2/scanner_state_machine.hpp"

// Pick a back-end
using ScannerStateMachine = msm::back::state_machine<ScannerProtocolDef>;

}  // namespace scanner_protocol
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_SCANNER_PROTOCOL_DEF_H
