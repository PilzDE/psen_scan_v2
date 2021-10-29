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

#include <chrono>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>

#include "psen_scan_v2/ros_integrationtest_helper.h"

using namespace std::chrono_literals;

namespace psen_scan_v2_test
{

class SubscriberMock
{
public:
  SubscriberMock()
  {
    subscriber_ = nh_.subscribe("/test_ns_laser_1/active_zoneset_marker", 10, &SubscriberMock::callback, this);
  }

  MOCK_METHOD1(callback, void(const visualization_msgs::Marker& event));

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
};

class ActiveZonesetNodeTest : public testing::Test
{
};

TEST_F(ActiveZonesetNodeTest, shouldAdvertiseZonesetTopic)
{
  EXPECT_TRUE(TopicExists("/test_ns_laser_1/active_zoneset_marker"));
}

}  // namespace psen_scan_v2_test

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "integrationtest_active_zoneset_node");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{ 1 };
  spinner.start();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
