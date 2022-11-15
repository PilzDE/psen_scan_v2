// Copyright (c) 2022 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_ENCODER_STATE_ROS_CONVERSIONS_H
#define PSEN_SCAN_V2_ENCODER_STATE_ROS_CONVERSIONS_H

#include <string>
#include <algorithm>

#include "psen_scan_v2/EncoderState.h"
#include "psen_scan_v2_standalone/encoder_state.h"

namespace psen_scan_v2
{

psen_scan_v2::EncoderState toEncoderStateMsg(const psen_scan_v2_standalone::EncoderState& encoder_state,
                                             const std::string& frame_id)
{
  psen_scan_v2::EncoderState ros_message;

  if (encoder_state.timestamp() < 0)
  {
    throw std::invalid_argument("EncoderState of Laserscan message has an invalid timestamp: " +
                                std::to_string(encoder_state.timestamp()));
  }

  ros_message.header.stamp = ros::Time{}.fromNSec(encoder_state.timestamp());
  ros_message.header.frame_id = frame_id;

  ros_message.encoder_1 = encoder_state.getEncoder1();
  ros_message.encoder_2 = encoder_state.getEncoder2();

  return ros_message;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ENCODER_STATE_ROS_CONVERSIONS_H
