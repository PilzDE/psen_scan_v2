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

#ifndef PSEN_SCAN_V2_ZONESET_MSG_BUILDER_H
#define PSEN_SCAN_V2_ZONESET_MSG_BUILDER_H

#include <cstdint>
#include <string>

#include <ros/time.h>
#include <geometry_msgs/Polygon.h>

#include "psen_scan_v2/ZoneSet.h"

namespace psen_scan_v2
{
class ZoneSetMsgBuilder
{
public:
  ZoneSet build() const;

public:
  ZoneSetMsgBuilder& headerSeq(uint32_t header_seq);
  ZoneSetMsgBuilder& headerStamp(const ros::Time& header_stamp);
  ZoneSetMsgBuilder& headerFrameId(const std::string& header_frame_id);
  ZoneSetMsgBuilder& safety1(const geometry_msgs::Polygon& safety1);
  ZoneSetMsgBuilder& safety2(const geometry_msgs::Polygon& safety2);
  ZoneSetMsgBuilder& safety3(const geometry_msgs::Polygon& safety3);
  ZoneSetMsgBuilder& warn1(const geometry_msgs::Polygon& warn1);
  ZoneSetMsgBuilder& warn2(const geometry_msgs::Polygon& warn2);
  ZoneSetMsgBuilder& muting1(const geometry_msgs::Polygon& muting1);
  ZoneSetMsgBuilder& muting2(const geometry_msgs::Polygon& muting2);
  ZoneSetMsgBuilder& safety1_Sub0(const geometry_msgs::Polygon& safety1_Sub0);
  ZoneSetMsgBuilder& safety2_Sub0(const geometry_msgs::Polygon& safety2_Sub0);
  ZoneSetMsgBuilder& safety3_Sub0(const geometry_msgs::Polygon& safety3_Sub0);
  ZoneSetMsgBuilder& warn1_Sub0(const geometry_msgs::Polygon& warn1_Sub0);
  ZoneSetMsgBuilder& warn2_Sub0(const geometry_msgs::Polygon& warn2_Sub0);
  ZoneSetMsgBuilder& muting1_Sub0(const geometry_msgs::Polygon& muting1_Sub0);
  ZoneSetMsgBuilder& muting2_Sub0(const geometry_msgs::Polygon& muting2_Sub0);
  ZoneSetMsgBuilder& safety1_Sub1(const geometry_msgs::Polygon& safety1_Sub1);
  ZoneSetMsgBuilder& safety2_Sub1(const geometry_msgs::Polygon& safety2_Sub1);
  ZoneSetMsgBuilder& safety3_Sub1(const geometry_msgs::Polygon& safety3_Sub1);
  ZoneSetMsgBuilder& warn1_Sub1(const geometry_msgs::Polygon& warn1_Sub1);
  ZoneSetMsgBuilder& warn2_Sub1(const geometry_msgs::Polygon& warn2_Sub1);
  ZoneSetMsgBuilder& muting1_Sub1(const geometry_msgs::Polygon& muting1_Sub1);
  ZoneSetMsgBuilder& muting2_Sub1(const geometry_msgs::Polygon& muting2_Sub1);
  ZoneSetMsgBuilder& safety1_Sub2(const geometry_msgs::Polygon& safety1_Sub2);
  ZoneSetMsgBuilder& safety2_Sub2(const geometry_msgs::Polygon& safety2_Sub2);
  ZoneSetMsgBuilder& safety3_Sub2(const geometry_msgs::Polygon& safety3_Sub2);
  ZoneSetMsgBuilder& warn1_Sub2(const geometry_msgs::Polygon& warn1_Sub2);
  ZoneSetMsgBuilder& warn2_Sub2(const geometry_msgs::Polygon& warn2_Sub2);
  ZoneSetMsgBuilder& muting1_Sub2(const geometry_msgs::Polygon& muting1_Sub2);
  ZoneSetMsgBuilder& muting2_Sub2(const geometry_msgs::Polygon& muting2_Sub2);
  ZoneSetMsgBuilder& speedLower(const float& speed_lower);
  ZoneSetMsgBuilder& speedUpper(const float& speed_upper);

private:
  ZoneSet zoneset_msg_;
};

inline ZoneSet ZoneSetMsgBuilder::build() const
{
  return zoneset_msg_;
}

inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::headerSeq(uint32_t seq)
{
  zoneset_msg_.header.seq = seq;
  return *this;
}

inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::headerStamp(const ros::Time& stamp)
{
  zoneset_msg_.header.stamp = stamp;
  return *this;
}

inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::headerFrameId(const std::string& frame_id)
{
  zoneset_msg_.header.frame_id = frame_id;
  return *this;
}

// ZoneSetMsgBuilder Master
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety1(const geometry_msgs::Polygon& safety1)
{
  zoneset_msg_.safety1 = safety1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety2(const geometry_msgs::Polygon& safety2)
{
  zoneset_msg_.safety2 = safety2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety3(const geometry_msgs::Polygon& safety3)
{
  zoneset_msg_.safety3 = safety3;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn1(const geometry_msgs::Polygon& warn1)
{
  zoneset_msg_.warn1 = warn1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn2(const geometry_msgs::Polygon& warn2)
{
  zoneset_msg_.warn2 = warn2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting1(const geometry_msgs::Polygon& muting1)
{
  zoneset_msg_.muting1 = muting1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting2(const geometry_msgs::Polygon& muting2)
{
  zoneset_msg_.muting2 = muting2;
  return *this;
}

// ZoneSetMsgBuilder Subscriber0
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety1_Sub0(const geometry_msgs::Polygon& safety1_Sub0)
{
  zoneset_msg_.safety1_Sub0 = safety1_Sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety2_Sub0(const geometry_msgs::Polygon& safety2_Sub0)
{
  zoneset_msg_.safety2_Sub0 = safety2_Sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety3_Sub0(const geometry_msgs::Polygon& safety3_Sub0)
{
  zoneset_msg_.safety3_Sub0 = safety3_Sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn1_Sub0(const geometry_msgs::Polygon& warn1_Sub0)
{
  zoneset_msg_.warn1_Sub0 = warn1_Sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn2_Sub0(const geometry_msgs::Polygon& warn2_Sub0)
{
  zoneset_msg_.warn2_Sub0 = warn2_Sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting1_Sub0(const geometry_msgs::Polygon& muting1_Sub0)
{
  zoneset_msg_.muting1_Sub0 = muting1_Sub0;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting2_Sub0(const geometry_msgs::Polygon& muting2_Sub0)
{
  zoneset_msg_.muting2_Sub0 = muting2_Sub0;
  return *this;
}

// ZoneSetMsgBuilder Subscriber1
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety1_Sub1(const geometry_msgs::Polygon& safety1_Sub1)
{
  zoneset_msg_.safety1_Sub1 = safety1_Sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety2_Sub1(const geometry_msgs::Polygon& safety2_Sub1)
{
  zoneset_msg_.safety2_Sub1 = safety2_Sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety3_Sub1(const geometry_msgs::Polygon& safety3_Sub1)
{
  zoneset_msg_.safety3_Sub1 = safety3_Sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn1_Sub1(const geometry_msgs::Polygon& warn1_Sub1)
{
  zoneset_msg_.warn1_Sub1 = warn1_Sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn2_Sub1(const geometry_msgs::Polygon& warn2_Sub1)
{
  zoneset_msg_.warn2_Sub1 = warn2_Sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting1_Sub1(const geometry_msgs::Polygon& muting1_Sub1)
{
  zoneset_msg_.muting1_Sub1 = muting1_Sub1;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting2_Sub1(const geometry_msgs::Polygon& muting2_Sub1)
{
  zoneset_msg_.muting2_Sub1 = muting2_Sub1;
  return *this;
}

// ZoneSetMsgBuilder Subscriber2
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety1_Sub2(const geometry_msgs::Polygon& safety1_Sub2)
{
  zoneset_msg_.safety1_Sub2 = safety1_Sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety2_Sub2(const geometry_msgs::Polygon& safety2_Sub2)
{
  zoneset_msg_.safety2_Sub2 = safety2_Sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::safety3_Sub2(const geometry_msgs::Polygon& safety3_Sub2)
{
  zoneset_msg_.safety3_Sub2 = safety3_Sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn1_Sub2(const geometry_msgs::Polygon& warn1_Sub2)
{
  zoneset_msg_.warn1_Sub2 = warn1_Sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::warn2_Sub2(const geometry_msgs::Polygon& warn2_Sub2)
{
  zoneset_msg_.warn2_Sub2 = warn2_Sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting1_Sub2(const geometry_msgs::Polygon& muting1_Sub2)
{
  zoneset_msg_.muting1_Sub2 = muting1_Sub2;
  return *this;
}
inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::muting2_Sub2(const geometry_msgs::Polygon& muting2_Sub2)
{
  zoneset_msg_.muting2_Sub2 = muting2_Sub2;
  return *this;
}


inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::speedLower(const float& speed_lower)
{
  zoneset_msg_.speed_lower = speed_lower;
  return *this;
}

inline ZoneSetMsgBuilder& ZoneSetMsgBuilder::speedUpper(const float& speed_upper)
{
  zoneset_msg_.speed_upper = speed_upper;
  return *this;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ZONESET_MSG_BUILDER_H
