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

#include <ros/ros.h>

#include "psen_scan_v2/config_server_node.h"
#include "psen_scan_v2/ZoneSetConfiguration.h"
#include "psen_scan_v2/zoneset_configuration_ros_conversion.h"
#include "psen_scan_v2_standalone/configuration/xml_configuration_parsing.h"
#include "psen_scan_v2_standalone/configuration/zoneset_configuration.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;

ConfigServerNode::ConfigServerNode(ros::NodeHandle& nh, const char* config_file_path, const std::string& frame_id)
  : nh_(nh)
{
  try
  {
    auto zoneconfig = configuration::xml_config_parsing::parseFile(config_file_path);
    zoneset_pub_ = nh_.advertise<::psen_scan_v2::ZoneSetConfiguration>(DEFAULT_ZONESET_TOPIC, 1, true /*latched*/);

    ROS_WARN_STREAM_NAMED(
        "ConfigurationServer",
        "The configuration server doesn't verfiy that the provided configuration file matches the one on the connected "
        "device! "
        "Mismatching configurations can amongst other things lead to confusing errors in navigation and misleading "
        "visualization. "
        "You are using \"" +
            std::string(config_file_path) + "\" please make sure that is the one you intented to use.");

    zoneset_pub_.publish(toRosMsg(zoneconfig, frame_id));
  }
  // LCOV_EXCL_START
  catch (const configuration::xml_config_parsing::XMLConfigurationParserException& e)
  {
    ROS_ERROR_STREAM_NAMED("ConfigServerNode", e.what());
  }
  // LCOV_EXCL_STOP
}
