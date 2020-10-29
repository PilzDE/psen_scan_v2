// Copyright (c) 2020 Pilz GmbH & Co. KG
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
#ifndef PSEN_SCAN_V2_MESSAGE_COLLECTOR_H
#define PSEN_SCAN_V2_MESSAGE_COLLECTOR_H

#include <future>

#include <ros/ros.h>

namespace psen_scan_v2_test
{
template <typename MsgType>
class MessageCollector
{
public:
  MessageCollector(ros::NodeHandle& nh) : nh_(nh){};

  typedef boost::shared_ptr<MsgType const> MsgTypeConstPtr;

  void scanCb(const MsgTypeConstPtr scan, size_t n_msgs)
  {
    if (msgs_.size() < n_msgs)
    {
      msgs_.push_back(scan);
    }

    // To have only one call on the promise the subscriber is shut down
    if (msgs_.size() == n_msgs)
    {
      sub_.shutdown();
      msgs_collected_.set_value();
    }
  }

  std::vector<MsgTypeConstPtr> collectScans(size_t n_msgs, std::string topic)
  {
    msgs_.clear();

    msgs_collected_ = std::promise<void>();

    auto future = msgs_collected_.get_future();
    sub_ = nh_.subscribe<MsgType>(
        topic, 1000, boost::bind(&MessageCollector::scanCb, this, boost::placeholders::_1, n_msgs));

    future.wait();

    return msgs_;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::vector<MsgTypeConstPtr> msgs_;
  std::promise<void> msgs_collected_;
};

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_MESSAGE_COLLECTOR_H