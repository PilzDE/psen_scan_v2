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

#include <functional>
#include <csignal>
#include <future>

#include <ros/ros.h>

#include "psen_scan_v2/function_pointers.h"
#include "psen_scan_v2/scanner.h"
#include "psen_scan_v2/scanner_controller.h"
#include "psen_scan_v2/ros_parameter_handler.h"
#include "psen_scan_v2/ros_scanner_node.h"
#include "psen_scan_v2/default_parameters.h"
#include "psen_scan_v2/scanner_configuration.h"
#include <rosconsole_bridge/bridge.h>
REGISTER_ROSCONSOLE_BRIDGE;

using namespace psen_scan_v2;

std::function<void()> NODE_TERMINATE_CB;

void delayed_shutdown_sig_handler(int sig)
{
  NODE_TERMINATE_CB();

  // Delay the shutdown() to get full debug output. Workaround for https://github.com/ros/ros_comm/issues/688
  ros::Duration(0.2).sleep();  // TODO check if we can get rid of this sleep

  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "psen_scan_v2_node");
  ros::NodeHandle pnh("~");

  std::signal(SIGINT, delayed_shutdown_sig_handler);

  try
  {
    psen_scan_v2::RosParameterHandler param_handler(pnh);

    ScannerConfiguration scanner_configuration(param_handler.getHostIP(),
                                               param_handler.getHostUDPPortData(),
                                               param_handler.getHostUDPPortControl(),
                                               param_handler.getSensorIP(),
                                               param_handler.getAngleStart(),
                                               param_handler.getAngleEnd());

    ROSScannerNode ros_scanner_node(pnh,
                                    DEFAULT_PUBLISH_TOPIC,
                                    param_handler.getFrameID(),
                                    param_handler.getXAxisRotation(),
                                    scanner_configuration);

    NODE_TERMINATE_CB = std::bind(&ROSScannerNode::terminate, &ros_scanner_node);

    auto f = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
    f.wait();
  }
  catch (std::exception& e)
  {
    ROS_ERROR_STREAM(e.what());
  }

  return 0;
}
