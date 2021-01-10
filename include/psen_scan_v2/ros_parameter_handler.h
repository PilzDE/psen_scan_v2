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

#ifndef PSEN_SCAN_V2_ROS_PARAMETER_HANDLER_H
#define PSEN_SCAN_V2_ROS_PARAMETER_HANDLER_H

#include <string>

#include <boost/optional.hpp>

#include <ros/ros.h>

#include "psen_scan_v2/get_ros_parameter_exception.h"

namespace psen_scan_v2
{
template <class T>
boost::optional<T> getParam(const ros::NodeHandle& node_handle, const std::string& key)
{
  if (!node_handle.hasParam(key))
  {
    ROS_WARN_STREAM("Parameter " + key + " doesn't exist on parameter server.");
    return boost::none;
  }

  T default_val{};
  boost::optional<T> ret_val(default_val);
  if (!node_handle.getParam(key, ret_val.get()))
  {
    throw WrongParameterType("Parameter " + key + " has wrong datatype on parameter server.");
  }
  return ret_val;
}

template <class T>
T getOptionalParamFromServer(const ros::NodeHandle& node_handle, const std::string& key, const T& default_value)
{
  boost::optional<T> val{ getParam<T>(node_handle, key) };
  if (!val)
  {
    return default_value;
  }
  return val.value();
}

template <class T>
T getRequiredParamFromServer(const ros::NodeHandle& node_handle, const std::string& key)
{
  boost::optional<T> val{ getParam<T>(node_handle, key) };
  if (!val)
  {
    throw ParamMissingOnServer("Parameter " + key + " doesn't exist on parameter server.");
  }
  return val.value();
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ROS_PARAMETER_HANDLER_H
