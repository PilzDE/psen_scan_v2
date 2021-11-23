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
/**
 * @brief Extension of psen_scan_v2_standalone::util::Barrier for tests involving ROS 2 nodes.
 *
 * This class allows to block execution of a thread until a callback of a rclcpp::Node triggers a release. The following
 * code example shows how to use this class for testing a method that should publish on a specific topic.
 *
 * \code
 * ACTION_P(OpenBarrier, barrier)
 * {
 *   barrier->release();
 * }
 *
 * TEST(MyNodeTests, methodUnderTestShouldPublishOnMyTopic)
 * {
 *   MyNode my_node;
 *   MyTopicSubscriberMock my_topic_sub; // child class of rclcpp::Node
 *
 *   RosBarrier barrier;
 *   EXPECT_CALL(my_topic_sub, callback()).WillOnce(OpenBarrier(&barrier));
 *
 *   my_node.methodUnderTest();
 *
 *   const std::chrono::seconds timeout{ 3 };
 *   barrier.spinUntilRelease(my_topic_sub, timeout)) << "methodUnderTest did not publish on my topic.";
 * }
 * \code
 */
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