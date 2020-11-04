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

#ifndef PSEN_SCAN_V2_UDP_CONNECTION_STATE_MACHINE_H
#define PSEN_SCAN_V2_UDP_CONNECTION_STATE_MACHINE_H

#include <functional>
#include <string>

#include <boost/core/demangle.hpp>

// back-end
#include <boost/msm/back/state_machine.hpp>
// front-end
#include <boost/msm/front/state_machine_def.hpp>

#include "psen_scan_v2/logging.h"
#include "psen_scan_v2/monitoring_frame_msg.h"
#include "psen_scan_v2/scanner_reply_msg.h"

namespace psen_scan_v2
{
template <class T>
inline std::string classNameShort(const T& t)
{
  const auto full_name{ boost::core::demangle(typeid(t).name()) };
  return full_name.substr(full_name.rfind("::") + 2);
}

namespace msm = boost::msm;
namespace mpl = boost::mpl;

// special case: lots of empty structs due to metaprogramming
// clang-format off


using MonitoringFrameCallback = std::function<void(const monitoring_frame::Message& msg)>;
using SendRequestCallback = std::function<void()>;
using StartedCallback = std::function<void()>;
using StoppedCallback = std::function<void()>;

// front-end: define the FSM structure
/**
 * @brief State machine implementing the scanner protocol.
 */
struct udp_connection_state_machine_ : public msm::front::state_machine_def<udp_connection_state_machine_>
{
  udp_connection_state_machine_(const MonitoringFrameCallback& monitoring_frame_cb,
                                const SendRequestCallback& start_request_cb,
                                const SendRequestCallback& stop_request_cb,
                                const StartedCallback& started_cb,
                                const StoppedCallback& stopped_cb)
    : monitoring_frame_callback_(monitoring_frame_cb)
    , send_start_request_callback_(start_request_cb)
    , send_stop_request_callback_(stop_request_cb)
    , notify_started_callback_(started_cb)
    , notify_stopped_callback_(stopped_cb)
  {
  }

  MonitoringFrameCallback monitoring_frame_callback_;
  SendRequestCallback send_start_request_callback_;
  SendRequestCallback send_stop_request_callback_;
  StoppedCallback notify_started_callback_;
  StoppedCallback notify_stopped_callback_;

  struct events
  {
    struct start_request {};
    struct reply_received
    {
      reply_received(ScannerReplyMsgType type) : type_(type) {}

      ScannerReplyMsgType type_;
    };
    struct monitoring_frame_received
    {
      monitoring_frame_received(const monitoring_frame::Message& frame) : frame_(frame) {}

      monitoring_frame::Message frame_;
    };
    struct start_reply_timeout {};
    struct stop_request {};
  };

  struct states
  {
    struct idle : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const& ,FSM&)
      {
        PSENSCAN_DEBUG("StateMachine", "Entering: IdleState");
      }
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& )
      {
        PSENSCAN_DEBUG("StateMachine", "Leaving: IdleState");
      }
    };

    struct wait_for_start_reply : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const& ,FSM&)
      {
        PSENSCAN_DEBUG("StateMachine", "Entering: WaitForStartReplyState");
      }
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& )
      {
        PSENSCAN_DEBUG("StateMachine", "Leaving: WaitForStartReplyState");
      }
    };

    struct wait_for_monitoring_frame : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const& ,FSM&)
      {
        PSENSCAN_DEBUG("StateMachine", "Entering: WaitForMonitoringFrames");
      }
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& )
      {
        PSENSCAN_DEBUG("StateMachine", "Leaving: WaitForMonitoringFrames");
      }
    };

    struct wait_for_stop_reply : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const& ,FSM&)
      {
        PSENSCAN_DEBUG("StateMachine", "Entering: WaitForStopReplyState");
      }
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& )
      {
        PSENSCAN_DEBUG("StateMachine", "Leaving: WaitForStopReplyState");
      }
    };

    struct stopped : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const& ,FSM&)
      {
        PSENSCAN_DEBUG("StateMachine", "Entering: Stopped");
      }
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& )
      {
        PSENSCAN_DEBUG("StateMachine", "Leaving: Stopped");
      }
    };

  };


  void action_send_start_request(events::start_request const&)
  {
    PSENSCAN_DEBUG("StateMachine", "Action: send_start_request_action");
    send_start_request_callback_();
  }

  void action_send_start_request(events::start_reply_timeout const&)
  {
    PSENSCAN_DEBUG("StateMachine", "Action: send_start_request_action");
    send_start_request_callback_();
  }

  void action_send_stop_request(events::stop_request const&)
  {
    PSENSCAN_DEBUG("StateMachine", "Action: send_stop_request_action");
    send_stop_request_callback_();
  }

  void action_notify_start(events::reply_received const&)
  {
    PSENSCAN_DEBUG("StateMachine", "Action: action_notify_start");
    notify_started_callback_();
  }

  void action_notify_stop(events::reply_received const&)
  {
    PSENSCAN_DEBUG("StateMachine", "Action: action_notify_stop");
    notify_stopped_callback_();
  }

  void action_handle_monitoring_frame(events::monitoring_frame_received const& event)
  {
    monitoring_frame_callback_(event.frame_);
  }

  bool guard_is_start_reply(events::reply_received const& reply_event)
  {
    return reply_event.type_ == ScannerReplyMsgType::start;
  }

  bool guard_is_stop_reply(events::reply_received const& reply_event)
  {
    return reply_event.type_ == ScannerReplyMsgType::stop;
  }

  typedef states::idle initial_state;

  typedef udp_connection_state_machine_ m;
  typedef events e;
  typedef states s;

  // Transition table for the scanner
  struct transition_table : mpl::vector<
    //    Start                                 Event                            Next           			           Action	                             Guard
    //  +---------------------------------+--------------------------------+-------------------------------+-----------------------------------+-----------------------------+
    a_row  < s::idle,                       e::start_request,                s::wait_for_start_reply,        &m::action_send_start_request                                 >,
    a_row  < s::idle,                       e::stop_request,                 s::wait_for_stop_reply,         &m::action_send_stop_request                                  >,
      row  < s::wait_for_start_reply,       e::reply_received,               s::wait_for_monitoring_frame,   &m::action_notify_start,            &m::guard_is_start_reply  >,
    a_irow < s::wait_for_monitoring_frame,  e::monitoring_frame_received,                                    &m::action_handle_monitoring_frame                            >,
    a_irow < s::wait_for_start_reply,       e::start_reply_timeout,                                          &m::action_send_start_request                                 >,
    a_row  < s::wait_for_start_reply,       e::stop_request,                 s::wait_for_stop_reply,         &m::action_send_stop_request                                  >,
    a_row  < s::wait_for_monitoring_frame,  e::stop_request,                 s::wait_for_stop_reply,         &m::action_send_stop_request                                  >,
      row  < s::wait_for_stop_reply,        e::reply_received,               s::stopped,                     &m::action_notify_stop,             &m::guard_is_stop_reply   >
    //  +---------------------------------+--------------------------------+-------------------------------+------------------------------------+-----------------------------+
  > {};
  // clang-format on

  // Replaces the default no-transition response.
  template <class FSM, class Event>
  void no_transition(Event const& event, FSM&, int state)
  {
    PSENSCAN_WARN("StateMachine", "No transition in state {} for event {}.", state, classNameShort(event));
  }

  template <class FSM>
  void no_transition(e::monitoring_frame_received const& event, FSM&, int state)
  {
    PSENSCAN_WARN("StateMachine", "Received monitoring frame despite not waiting for it (in State {})", state);
  }
};

// Pick a back-end
typedef msm::back::state_machine<udp_connection_state_machine_> udp_connection_state_machine;

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_UDP_CONNECTION_STATE_MACHINE_H
