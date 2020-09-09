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


using SendStartRequestCallback = std::function<void()>;

// front-end: define the FSM structure
struct udp_connection_state_machine_ : public msm::front::state_machine_def<udp_connection_state_machine_>
{
  udp_connection_state_machine_(const SendStartRequestCallback& sr):
    send_start_request_callback_(sr)
  {
  }

  SendStartRequestCallback send_start_request_callback_;

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
    struct init : public msm::front::state<>
    {
      template <class Event,class FSM>
      void on_entry(Event const& ,FSM&)
      {
        PSENSCAN_DEBUG("StateMachine", "Entering: InitState");
      }
      template <class Event,class FSM>
      void on_exit(Event const&,FSM& )
      {
        PSENSCAN_DEBUG("StateMachine", "Leaving: InitState");
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
      // TODO Add logging
    };

    struct wait_for_stop_reply : public msm::front::state<>
    {
      // TODO Add logging
    };
  };


  void action_send_start_request(events::start_request const&)
  {
    PSENSCAN_DEBUG("StateMachine", "Action: send_start_request_action");
    send_start_request_callback_();
  }

  typedef states::init initial_state;
  //typedef typename states::init initial_state;

  typedef udp_connection_state_machine_ m;  // makes transition table cleaner
  typedef events e;  // makes transition table cleaner
  typedef states s;

  // Transition table for the scanner
  struct transition_table : mpl::vector<
    //    Start                                 Event                            Next           			           Action	                             Guard
    //  +--------------------------------+--------------------------------+-------------------------------+-----------------------------------+-------+
    a_row < s::init,                       e::start_request,                 s::wait_for_start_reply,      &m::action_send_start_request   >,
     _row < s::wait_for_start_reply,       e::start_reply_received,          s::wait_for_monitoring_frame                                  >,
     _row < s::wait_for_monitoring_frame,  e::monitoring_frame_received,     s::wait_for_monitoring_frame                                  >,
     _row < s::wait_for_monitoring_frame,  e::stop_request,                  s::wait_for_stop_reply                                        >,
     _row < s::wait_for_stop_reply,        e::stop_reply_received,           s::init                                                       >
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
