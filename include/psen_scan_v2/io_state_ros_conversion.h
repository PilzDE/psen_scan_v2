// Copyright (c) 2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_IO_STATE_ROS_CONVERSIONS_H
#define PSEN_SCAN_V2_IO_STATE_ROS_CONVERSIONS_H

#include <string>
#include <algorithm>

#include "psen_scan_v2/IOState.h"
#include "psen_scan_v2/InputPinState.h"
#include "psen_scan_v2/OutputPinState.h"
#include "psen_scan_v2_standalone/io_state.h"

namespace psen_scan_v2
{
template <typename PinStateType>
PinStateType toPinStateMsg(const psen_scan_v2_standalone::PinState& pin)
{
  PinStateType pin_msg;
  pin_msg.pin_id.id = pin.id();
  pin_msg.name = pin.name();
  pin_msg.state = pin.state();
  return pin_msg;
}

psen_scan_v2::IOState toIOStateMsg(const psen_scan_v2_standalone::IOState& io_state,
                                   const std::string& frame_id,
                                   int64_t stamp)
{
  psen_scan_v2::IOState ros_message;
  if (stamp < 0)
  {
    throw std::invalid_argument("Laserscan message has an invalid timestamp: " + std::to_string(stamp));
  }
  ros_message.header.stamp = ros::Time{}.fromNSec(stamp);
  ros_message.header.frame_id = frame_id;

  auto input = io_state.input();
  std::transform(input.begin(), input.end(), std::back_inserter(ros_message.input), [](const auto& pin) {
    return toPinStateMsg<InputPinState>(pin);
  });

  auto output = io_state.output();
  std::transform(output.begin(), output.end(), std::back_inserter(ros_message.output), [](const auto& pin) {
    return toPinStateMsg<OutputPinState>(pin);
  });

  return ros_message;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_IO_STATE_ROS_CONVERSIONS_H
