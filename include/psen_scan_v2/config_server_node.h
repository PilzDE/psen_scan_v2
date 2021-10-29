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

#ifndef PSEN_SCAN_V2_CONFIG_SERVER_NODE_H
#define PSEN_SCAN_V2_CONFIG_SERVER_NODE_H

#include <string>

#include <gtest/gtest_prod.h>

#include <ros/ros.h>

#include "psen_scan_v2/ZoneSet.h"
#include "psen_scan_v2/ZoneSetConfiguration.h"

/**
 * @brief Root namespace for the ROS part
 */
namespace psen_scan_v2
{
static const std::string DEFAULT_ZONESET_TOPIC = "zoneconfiguration";

/**
 * @brief ROS Node that publishes a latched topic containing the configured zonesets.
 *
 */
class ConfigServerNode
{
public:
  ConfigServerNode(ros::NodeHandle& nh, const char* config_file_path, const std::string& frame_id);

private:
  ros::NodeHandle nh_;
  ros::Publisher zoneset_pub_;
};
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_CONFIG_SERVER_NODE_H
