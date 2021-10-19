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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "config_server_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh{ "~" };

  std::string config_file;
  if (pnh.getParam("config_file", config_file))
  {
    psen_scan_v2::ConfigServerNode config_server_node(nh, config_file.c_str());

    ros::spin();
  }
  // LCOV_EXCL_START
  else
  {
    ROS_ERROR("PARAMETER config_file not set.");
  }
  // LCOV_EXCL_STOP
}