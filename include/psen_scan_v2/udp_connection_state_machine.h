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

// back-end
#include <boost/msm/back/state_machine.hpp>
// front-end
#include <boost/msm/front/state_machine_def.hpp>

#include "psen_scan_v2/logging.h"

namespace psen_scan_v2
{
namespace msm = boost::msm;
namespace mpl = boost::mpl;

// special case: lots of empty structs due to metaprogramming
// clang-format off


using SendRequestCallback = std::function<void()>;
using StoppedCallback = std::function<void()>;

// front-end: define the FSM structure
/**
 * @brief State machine implementing the scanner protocol.
 */
struct udp_connection_state_machine_ : public msm::front::state_machine_def<udp_connection_state_machine_>
{
  udp_connection_state_machine_(const SendRequestCallback& start_request_cb,
                                const SendRequestCallback& stop_request_cb,
                                const StoppedCallback& stopped_cb)
    : send_start_request_callback_(start_request_cb)
    , send_stop_request_callback_(stop_request_cb)
    , notify_stopped_callback_(stopped_cb)
  {
  }

  SendRequestCallback send_start_request_callback_;
  SendRequestCallback send_stop_request_callback_;
  StoppedCallback notify_stopped_callback_;

  struct events
  {
    struct start_request {};
    struct start_reply_received {};
    struct monitoring_frame_received {};
    struct stop_request {};
    struct stop_reply_received {};
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
        // This will block all ROS calls since the state is entered/left repeatedly
        //PSENSCAN_DEBUG("StateMachine", "Entering: WaitForStopReplyState"); // TODO find solution
      }
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& )
      {
        // This will block all ROS calls since the state is entered/left repeatedly
        //PSENSCAN_DEBUG("StateMachine", "Leaving: WaitForStopReplyState"); // TODO find solution
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

  void action_send_stop_request(events::stop_request const&)
  {
    PSENSCAN_DEBUG("StateMachine", "Action: send_stop_request_action");
    send_stop_request_callback_();
  }

  void action_notify_stop(events::stop_reply_received const&)
  {
    PSENSCAN_DEBUG("StateMachine", "Action: action_notify_stop");
    notify_stopped_callback_();
  }

  typedef states::idle initial_state;
  //typedef typename states::idle initial_state;

  typedef udp_connection_state_machine_ m;  // makes transition table cleaner
  typedef events e;  // makes transition table cleaner
  typedef states s;

  // Transition table for the scanner
  struct transition_table : mpl::vector<
    //    Start                                 Event                            Next           			           Action	                             Guard
    //  +--------------------------------+--------------------------------+-------------------------------+-----------------------------------+-------+
    a_row < s::idle,                       e::start_request,                 s::wait_for_start_reply,      &m::action_send_start_request   >,
    a_row < s::idle,                       e::stop_request,                  s::wait_for_stop_reply,       &m::action_send_stop_request    >,
     _row < s::wait_for_start_reply,       e::start_reply_received,          s::wait_for_monitoring_frame                                  >,
     _row < s::wait_for_monitoring_frame,  e::monitoring_frame_received,     s::wait_for_monitoring_frame                                  >,
    a_row < s::wait_for_start_reply,       e::stop_request,                  s::wait_for_stop_reply,       &m::action_send_stop_request    >,
    a_row < s::wait_for_monitoring_frame,  e::stop_request,                  s::wait_for_stop_reply,       &m::action_send_stop_request    >,
    a_row < s::wait_for_stop_reply,        e::stop_reply_received,           s::stopped,                   &m::action_notify_stop          >
    //  +--------------------------------+--------------------------------+------------------------------------+------------------------------------+-------+
  > {};
  // clang-format on

  // LCOV_EXCL_START
  // TODO: Activate coverage again when function is actually used
  // Replaces the default no-transition response.
  template <class FSM, class Event>
  void no_transition(Event const&, FSM&, int)
  {
    // TODO Implement handling
  }
  // LCOV_EXCL_STOP
};

// Pick a back-end
typedef msm::back::state_machine<udp_connection_state_machine_> udp_connection_state_machine;

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_UDP_CONNECTION_STATE_MACHINE_H
