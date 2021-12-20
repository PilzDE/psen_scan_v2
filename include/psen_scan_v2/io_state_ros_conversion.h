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

#include "psen_scan_v2/IOState.h"
#include "psen_scan_v2/PinState.h"
#include "psen_scan_v2_standalone/io_state.h"

namespace psen_scan_v2
{
psen_scan_v2::PinState toPinStateMsg(const psen_scan_v2_standalone::PinState& pin)
{
  psen_scan_v2::PinState pin_msg;
  pin_msg.pin_id = pin.id();
  pin_msg.name = pin.name();
  pin_msg.state = pin.state();
  return pin_msg;
}

std::vector<psen_scan_v2::PinState> readPinStates(const std::vector<psen_scan_v2_standalone::PinState>& pins)
{
  std::vector<psen_scan_v2::PinState> container;
  container.reserve(pins.size());
  for (const auto& pin : pins)
  {
    container.emplace_back(toPinStateMsg(pin));
  }

  return container;
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

  ros_message.logical_input = readPinStates(io_state.logicalInput());
  ros_message.output = readPinStates(io_state.output());

  return ros_message;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_IO_STATE_ROS_CONVERSIONS_H
