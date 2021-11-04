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
#ifndef PSEN_SCAN_V2_ROS_INTEGRATIONTEST_HELPER_H
#define PSEN_SCAN_V2_ROS_INTEGRATIONTEST_HELPER_H

#include <chrono>
#include <string>
#include <sstream>
#include <thread>

#include <gtest/gtest.h>

#include <ros/master.h>

using namespace std::chrono_literals;

namespace psen_scan_v2_test
{
::testing::AssertionResult TopicExists(const std::string& topic,
                                       const std::chrono::milliseconds& sleep_after_fail = 500ms,
                                       size_t retries = 10)
{
  std::string topic_names;  // For verbose information on failure

  for (size_t i = 0; i < retries; i++)
  {
    if (i > 0)
    {
      std::this_thread::sleep_for(sleep_after_fail);
    }

    std::stringstream ss_topic_names;

    ros::master::V_TopicInfo available_topics;
    ros::master::getTopics(available_topics);

    for (ros::master::V_TopicInfo::iterator it = available_topics.begin(); it != available_topics.end(); it++)
    {
      const ros::master::TopicInfo& info = *it;
      if (info.name == topic)
      {
        return ::testing::AssertionSuccess();
      }
      ss_topic_names << "\"" << info.name << "\" ";
    }
    topic_names = ss_topic_names.str();
  }

  return ::testing::AssertionFailure() << "Topic \"" << topic << "\" not found. Available topics: " << topic_names;
}

/**
 * @brief Blocks until a node defined by node_name comes up.
 * @param node_name The name of the node to wait for.
 * @param loop_frequency Frequency at which the system is checked for the node.
 */
inline ::testing::AssertionResult NodeExists(const std::string node_name,
                                             const double loop_frequency = 10.0,
                                             const ros::Duration timeout = ros::Duration(10.0))
{
  ROS_INFO_STREAM("Waiting for Node " << node_name);
  std::vector<std::string> node_names;
  std::stringstream ss_node_names;
  const auto timeout_time = ros::Time::now() + timeout;
  if (ros::master::getNodes(node_names))
  {
    for (auto node : node_names)
    {
      ss_node_names << "\"" << node << "\" ";
    }
  }

  while (ros::master::getNodes(node_names) &&
         std::find(node_names.begin(), node_names.end(), node_name) == node_names.end())
  {
    if (ros::Time::now() > timeout_time)
    {
      return ::testing::AssertionFailure() << "Timeout while waiting for node: \"" << node_name << "\""
                                           << "\". Available nodes: " << ss_node_names.str();
    }
    ros::Rate(loop_frequency).sleep();
  }

  return ::testing::AssertionSuccess() << "Found node \"" << node_name;
}

MATCHER(isLatched, "")
{
  auto connection_header = arg.getConnectionHeader();
  auto search = connection_header.find("latching");
  return (search != connection_header.end() && search->second == "1");
}

MATCHER_P(messageEQ, expected_msg, "")
{
  auto actual_msg = arg.getMessage();
  return expected_msg == *actual_msg;
}

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_ROS_INTEGRATIONTEST_HELPER_H
