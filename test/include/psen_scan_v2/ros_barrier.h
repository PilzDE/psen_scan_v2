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

#ifndef PSEN_SCAN_V2_TEST_ROS_BARRIER_H
#define PSEN_SCAN_V2_TEST_ROS_BARRIER_H

#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "psen_scan_v2_standalone/util/async_barrier.h"

namespace psen_scan_v2_test
{
class RosBarrier : public psen_scan_v2_standalone::util::Barrier
{
public:
  template <class Rep, class Period>
  rclcpp::FutureReturnCode spinUntilRelease(rclcpp::Node::SharedPtr node,
                                            const std::chrono::duration<Rep, Period>& timeout) const;
};

template <class Rep, class Period>
inline rclcpp::FutureReturnCode RosBarrier::spinUntilRelease(rclcpp::Node::SharedPtr node,
                                                             const std::chrono::duration<Rep, Period>& timeout) const
{
  return rclcpp::spin_until_future_complete(node, this->future_, timeout);
}
}  // namespace psen_scan_v2_test

#endif