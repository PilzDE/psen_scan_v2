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

#include "psen_scan_v2/active_zoneset_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "active_zoneset_node");
  ros::NodeHandle nh;

  try
  {
    psen_scan_v2::ActiveZonesetNode active_zoneset_node{ nh };
    ros::spin();
  }
  // LCOV_EXCL_START
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
    return 1;
  }
  // LCOV_EXCL_STOP

  return 0;
}
