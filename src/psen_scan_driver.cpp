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

#include <chrono>
#include <csignal>
#include <functional>
#include <future>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "psen_scan_v2_standalone/configuration/default_parameters.h"
#include "psen_scan_v2_standalone/protocol_layer/function_pointers.h"
#include "psen_scan_v2_standalone/scanner_configuration.h"
#include "psen_scan_v2_standalone/scanner_config_builder.h"
#include "psen_scan_v2_standalone/scan_range.h"

#include "psen_scan_v2/ros_parameter_handler.h"
#include "psen_scan_v2/ros_scanner_node.h"

using namespace psen_scan_v2;
using namespace psen_scan_v2_standalone::configuration;
using namespace std::chrono_literals;

std::function<void()> NODE_TERMINATE_CALLBACK;

const std::string PARAM_HOST_IP{ "host_ip" };
const std::string PARAM_HOST_DATA_PORT{ "host_udp_port_data" };
const std::string PARAM_HOST_CONTROL_PORT{ "host_udp_port_control" };
const std::string PARAM_SCANNER_IP{ "sensor_ip" };
const std::string PARAM_TF_PREFIX{ "tf_prefix" };
const std::string PARAM_ANGLE_START{ "angle_start" };
const std::string PARAM_ANGLE_END{ "angle_end" };
const std::string PARAM_X_AXIS_ROTATION{ "x_axis_rotation" };
const std::string PARAM_FRAGMENTED_SCANS{ "fragmented_scans" };
const std::string PARAM_INTENSITIES{ "intensities" };
const std::string PARAM_RESOLUTION{ "resolution" };

static const std::string DEFAULT_TF_PREFIX = "laser_1";

//! @brief Topic on which the LaserScan data are published.
static const std::string DEFAULT_PUBLISH_TOPIC = "scan";

void shutdown_sig_handler(int sig)
{
  NODE_TERMINATE_CALLBACK();
  rclcpp::shutdown();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("psen_scan_v2_node", node_options);

  std::signal(SIGINT, shutdown_sig_handler);

  try
  {
    ScanRange scan_range{ util::TenthOfDegree::fromRad(
                              configuration::DEFAULT_X_AXIS_ROTATION +
                              getOptionalParam<double>(*node, PARAM_ANGLE_START, configuration::DEFAULT_ANGLE_START)),
                          util::TenthOfDegree::fromRad(
                              configuration::DEFAULT_X_AXIS_ROTATION +
                              getOptionalParam<double>(*node, PARAM_ANGLE_END, configuration::DEFAULT_ANGLE_END)) };

    ScannerConfiguration scanner_configuration{
      ScannerConfigurationBuilder()
          .hostIP(getOptionalParam<std::string>(*node, PARAM_HOST_IP, configuration::DEFAULT_HOST_IP_STRING))
          .hostDataPort(getOptionalParam<int>(*node, PARAM_HOST_DATA_PORT, configuration::DATA_PORT_OF_HOST_DEVICE))
          .hostControlPort(
              getOptionalParam<int>(*node, PARAM_HOST_CONTROL_PORT, configuration::CONTROL_PORT_OF_HOST_DEVICE))
          .scannerIp(getRequiredParam<std::string>(*node, PARAM_SCANNER_IP))
          .scannerDataPort(configuration::DATA_PORT_OF_SCANNER_DEVICE)
          .scannerControlPort(configuration::CONTROL_PORT_OF_SCANNER_DEVICE)
          .scanRange(scan_range)
          .enableDiagnostics()
          .enableFragmentedScans(getOptionalParam<bool>(*node, PARAM_FRAGMENTED_SCANS, configuration::FRAGMENTED_SCANS))
          .enableIntensities(getOptionalParam<bool>(*node, PARAM_INTENSITIES, configuration::INTENSITIES))
          .scanResolution(util::TenthOfDegree::fromRad(
              getOptionalParam<double>(*node, PARAM_RESOLUTION, configuration::DEFAULT_SCAN_ANGLE_RESOLUTION)))
          .build()
    };

    if (scanner_configuration.fragmentedScansEnabled())
    {
      RCLCPP_INFO(node->get_logger(), "Using fragmented scans.");
    }

    ROSScannerNode ros_scanner_node(node,
                                    DEFAULT_PUBLISH_TOPIC,
                                    getOptionalParam<std::string>(*node, PARAM_TF_PREFIX, DEFAULT_TF_PREFIX),
                                    configuration::DEFAULT_X_AXIS_ROTATION,
                                    scanner_configuration);

    NODE_TERMINATE_CALLBACK = std::bind(&ROSScannerNode::terminate, &ros_scanner_node);

    auto f = std::async(std::launch::async, [&ros_scanner_node]() { ros_scanner_node.run(); });
    f.wait();
  }
  catch (std::exception& e)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
  }

  return 0;
}
