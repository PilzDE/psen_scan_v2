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

#include <functional>
#include <csignal>
#include <future>
#include <string>

#include <ros/ros.h>

#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/protocol_layer/function_pointers.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scanner_config_builder.h"
#include "psen_scan_v2_standalone/scan_range.h"

#include "psen_scan_v2/ros_parameter_handler.h"
#include "psen_scan_v2/ros_scanner_node.h"

#include <rosconsole_bridge/bridge.h>
REGISTER_ROSCONSOLE_BRIDGE;

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone;
using namespace psen_scan_v2_standalone::configuration;

std::function<void()> NODE_TERMINATE_CB;

const std::string PARAM_HOST_IP{ "host_ip" };
const std::string PARAM_HOST_DATA_PORT{ "host_udp_port_data" };
const std::string PARAM_HOST_CONTROL_PORT{ "host_udp_port_control" };
const std::string PARAM_SCANNER_IP{ "sensor_ip" };
const std::string PARAM_PREFIX{ "prefix" };
const std::string PARAM_ANGLE_START{ "angle_start" };
const std::string PARAM_ANGLE_END{ "angle_end" };
const std::string PARAM_X_AXIS_ROTATION{ "x_axis_rotation" };
const std::string PARAM_FRAGMENTED_SCANS{ "fragmented_scans" };
const std::string PARAM_INTENSITIES{ "intensities" };
const std::string PARAM_RESOLUTION{ "resolution" };

static const std::string DEFAULT_PREFIX = "scanner";

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
    ScanRange scan_range{ util::TenthOfDegree::fromRad(configuration::DEFAULT_X_AXIS_ROTATION +
                                                       getOptionalParamFromServer<double>(
                                                           pnh, PARAM_ANGLE_START, configuration::DEFAULT_ANGLE_START)),
                          util::TenthOfDegree::fromRad(configuration::DEFAULT_X_AXIS_ROTATION +
                                                       getOptionalParamFromServer<double>(
                                                           pnh, PARAM_ANGLE_END, configuration::DEFAULT_ANGLE_END)) };

    ScannerConfiguration scanner_configuration{
      ScannerConfigurationBuilder()
          .hostIP(getOptionalParamFromServer<std::string>(pnh, PARAM_HOST_IP, configuration::DEFAULT_HOST_IP_STRING))
          .hostDataPort(
              getOptionalParamFromServer<int>(pnh, PARAM_HOST_DATA_PORT, configuration::DATA_PORT_OF_HOST_DEVICE))
          .hostControlPort(
              getOptionalParamFromServer<int>(pnh, PARAM_HOST_CONTROL_PORT, configuration::CONTROL_PORT_OF_HOST_DEVICE))
          .scannerIp(getRequiredParamFromServer<std::string>(pnh, PARAM_SCANNER_IP))
          .scannerDataPort(configuration::DATA_PORT_OF_SCANNER_DEVICE)
          .scannerControlPort(configuration::CONTROL_PORT_OF_SCANNER_DEVICE)
          .scanRange(scan_range)
          .enableDiagnostics()
          .enableFragmentedScans(
              getOptionalParamFromServer<bool>(pnh, PARAM_FRAGMENTED_SCANS, configuration::FRAGMENTED_SCANS))
          .enableIntensities(getOptionalParamFromServer<bool>(pnh, PARAM_INTENSITIES, configuration::INTENSITIES))
          .scanResolution(util::TenthOfDegree::fromRad(
              getOptionalParamFromServer<double>(pnh, PARAM_RESOLUTION, configuration::DEFAULT_SCAN_ANGLE_RESOLUTION)))
          .build()
    };

    if (scanner_configuration.fragmentedScansEnabled())
    {
      ROS_INFO("Using fragmented scans.");
    }

    ROSScannerNode ros_scanner_node(pnh,
                                    DEFAULT_PUBLISH_TOPIC,
                                    getOptionalParamFromServer<std::string>(pnh, PARAM_PREFIX, DEFAULT_PREFIX),
                                    configuration::DEFAULT_X_AXIS_ROTATION,
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
