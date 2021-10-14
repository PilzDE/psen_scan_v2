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

#include <rclcpp/node.hpp>

#include "psen_scan_v2/get_ros_parameter_exception.h"

namespace psen_scan_v2
{
/**
 * @throws rclcpp::exceptions::InvalidParameterTypeException if the requested type does not match the stored parameter.
 */
template <class T>
T getOptionalParam(const rclcpp::Node& node, const std::string& key, const T& default_value)
{
  T ret_val{};
  if (!node.get_parameter_or(key, ret_val, default_value))
  {
    RCLCPP_WARN_STREAM(node.get_logger(), "Parameter " + key + " doesn't exist for node " + node.get_name() + ".");
  }
  return ret_val;
}

/**
 * @throws ParamMissingOnServer if the parameter was not set.
 * @throws rclcpp::exceptions::InvalidParameterTypeException if the requested type does not match the stored parameter.
 */
template <class T>
T getRequiredParam(const rclcpp::Node& node, const std::string& key)
{
  T ret_val{};
  if (!node.get_parameter(key, ret_val))
  {
    throw ParamMissingOnServer("Parameter " + key + " doesn't exist for node " + node.get_name() + ".");
  }
  return ret_val;
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ROS_PARAMETER_HANDLER_H
