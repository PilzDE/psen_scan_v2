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
#ifndef PSEN_SCAN_V2_SUBSCRIBER_MOCK_H
#define PSEN_SCAN_V2_SUBSCRIBER_MOCK_H

#include <string>

#include <gtest/gtest.h>

#include <ros/master.h>

namespace psen_scan_v2_test
{
template <typename T>
class SubscriberMock
{
public:
  SubscriberMock(ros::NodeHandle& nh, std::string topicName, int queueSize)
  {
    subscriber_ = nh.subscribe(topicName, queueSize, &SubscriberMock::callback, this);
  }

  MOCK_METHOD1_T(callback, void(const T& msg));

  ros::Subscriber getSubscriber()
  {
    return subscriber_;
  }

private:
  ros::Subscriber subscriber_;
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_SUBSCRIBER_MOCK_H
