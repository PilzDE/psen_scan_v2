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
#include <string>

//#include <fmt/ostream.h>

#include <ros/ros.h>

#include "psen_scan_v2/function_pointers.h"
#include "psen_scan_v2/scanner.h"
#include "psen_scan_v2/scanner_controller.h"
#include "psen_scan_v2/ros_parameter_handler.h"
#include "psen_scan_v2/ros_scanner_node.h"
#include "psen_scan_v2/default_parameters.h"
#include "psen_scan_v2/scanner_configuration.h"
#include "psen_scan_v2/scan_range.h"

#include <rosconsole_bridge/bridge.h>
REGISTER_ROSCONSOLE_BRIDGE;

using namespace psen_scan_v2;

std::function<void()> NODE_TERMINATE_CB;

const std::string PARAM_HOST_IP{ "host_ip" };
const std::string PARAM_HOST_DATA_PORT{ "host_udp_port_data" };
const std::string PARAM_HOST_CONTROL_PORT{ "host_udp_port_control" };
const std::string PARAM_SCANNER_IP{ "sensor_ip" };
const std::string PARAM_FRAME_ID{ "frame_id" };
const std::string PARAM_ANGLE_START{ "angle_start" };
const std::string PARAM_ANGLE_END{ "angle_end" };
const std::string PARAM_X_AXIS_ROTATION{ "x_axis_rotation" };

static const std::string DEFAULT_FRAME_ID = "scanner";

//! @brief Topic on which the LaserScan data are published.
static const std::string DEFAULT_PUBLISH_TOPIC = "scan";

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
    DefaultScanRange scan_range{
      TenthOfDegree::fromRad(DEFAULT_X_AXIS_ROTATION -
                             getOptionalParamFromServer<double>(pnh, PARAM_ANGLE_END, DEFAULT_ANGLE_END)),
      TenthOfDegree::fromRad(DEFAULT_X_AXIS_ROTATION -
                             getOptionalParamFromServer<double>(pnh, PARAM_ANGLE_START, DEFAULT_ANGLE_START))
    };

    ScannerConfiguration scanner_configuration(getRequiredParamFromServer<std::string>(pnh, PARAM_HOST_IP),
                                               getRequiredParamFromServer<int>(pnh, PARAM_HOST_DATA_PORT),
                                               getRequiredParamFromServer<int>(pnh, PARAM_HOST_CONTROL_PORT),
                                               getRequiredParamFromServer<std::string>(pnh, PARAM_SCANNER_IP),
                                               scan_range,
                                               true);

    ROSScannerNode ros_scanner_node(pnh,
                                    DEFAULT_PUBLISH_TOPIC,
                                    getOptionalParamFromServer<std::string>(pnh, PARAM_FRAME_ID, DEFAULT_FRAME_ID),
                                    DEFAULT_X_AXIS_ROTATION,
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
