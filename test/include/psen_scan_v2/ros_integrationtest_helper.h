// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

using ::testing::Eq;
using ::testing::Matches;
using ::testing::SizeIs;

MATCHER_P(messageZoneSetCountEQ, expected_msg, "")
{
  auto actual_msg = arg.getMessage();
  return Matches(SizeIs(expected_msg.zonesets.size()))(actual_msg->zonesets);
}

MATCHER_P(zonesetSpeedLimitsEQ, expected_zoneset, "")
{
  return Matches(Eq(expected_zoneset.speed_lower))(arg.speed_lower) &&
         Matches(Eq(expected_zoneset.speed_upper))(arg.speed_upper);
}

MATCHER_P(zonesetVecSpeedLimitsEQ, expected_zonesets, "")
{
  return std::equal(
      expected_zonesets.begin(), expected_zonesets.end(), arg.begin(), arg.end(), [](const auto& a, const auto& b) {
        return Matches(zonesetSpeedLimitsEQ(a))(b);
      });
}

MATCHER_P(messageZoneSetsSpeedLimitsEQ, expected_msg, "")
{
  auto actual_msg = arg.getMessage();
  return Matches(zonesetVecSpeedLimitsEQ(expected_msg.zonesets))(actual_msg->zonesets);
}

MATCHER_P(zonesetVecFrameIdsEQ, expected_zonesets, "")
{
  return std::equal(
      expected_zonesets.begin(), expected_zonesets.end(), arg.begin(), arg.end(), [](const auto& a, const auto& b) {
        return Matches(Eq(a.header.frame_id))(b.header.frame_id);
      });
}

MATCHER_P(messageZoneSetsFrameIdEQ, expected_msg, "")
{
  auto actual_msg = arg.getMessage();
  return Matches(zonesetVecFrameIdsEQ(expected_msg.zonesets))(actual_msg->zonesets);
}

MATCHER_P(zonesetPolygonPointCountsEQ, expected_zoneset, "")
{
  return Matches(SizeIs(expected_zoneset.safety1.points.size()))(arg.safety1.points) &&
         Matches(SizeIs(expected_zoneset.safety2.points.size()))(arg.safety2.points) &&
         Matches(SizeIs(expected_zoneset.safety3.points.size()))(arg.safety3.points) &&
         Matches(SizeIs(expected_zoneset.warn1.points.size()))(arg.warn1.points) &&
         Matches(SizeIs(expected_zoneset.warn2.points.size()))(arg.warn2.points) &&
         Matches(SizeIs(expected_zoneset.muting1.points.size()))(arg.muting1.points) &&
         Matches(SizeIs(expected_zoneset.muting2.points.size()))(arg.muting2.points);
}

MATCHER_P(zonesetVecPolygonPointCountsEQ, expected_zonesets, "")
{
  return std::equal(
      expected_zonesets.begin(), expected_zonesets.end(), arg.begin(), arg.end(), [](const auto& a, const auto& b) {
        return Matches(zonesetPolygonPointCountsEQ(a))(b);
      });
}

MATCHER_P(messageZoneSetsPolygonPointCountsEQ, expected_msg, "")
{
  auto actual_msg = arg.getMessage();
  return Matches(zonesetVecPolygonPointCountsEQ(expected_msg.zonesets))(actual_msg->zonesets);
}

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_ROS_INTEGRATIONTEST_HELPER_H