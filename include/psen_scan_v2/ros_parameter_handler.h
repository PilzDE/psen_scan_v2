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

#ifndef PSEN_SCAN_V2_ROS_PARAMETER_HANDLER_H
#define PSEN_SCAN_V2_ROS_PARAMETER_HANDLER_H

#include <string>

#include <ros/ros.h>

namespace psen_scan_v2
{
/**
 * @brief Class for getting ROS-Parameters from the parameter-server
 *
 */
class RosParameterHandler
{
public:
  RosParameterHandler(const ros::NodeHandle& nh);
  void updateAllParamsFromParamServer();
  template <class T>
  void getRequiredParamFromParamServer(const std::string& key, T& param);
  template <class T>
  bool getOptionalParamFromParamServer(const std::string& key, T& param);
  std::string getHostIP() const;
  uint32_t getHostUDPPortData() const;
  uint32_t getHostUDPPortControl() const;
  std::string getSensorIP() const;
  std::string getFrameID() const;
  double getAngleStart() const;
  double getAngleEnd() const;
  double getXAxisRotation() const;

private:
  ros::NodeHandle const nh_;  /**< Nodehandle through which parameters are fetched */
  std::string host_ip_;       /**< IP-Adress of host machine */
  int host_udp_port_data_;    /**< UDP Port on which monitoring frames (scans) should be received.*/
  int host_udp_port_control_; /**< UDP Port used to send commands (start/stop) and receive the corresponding replies. */
  std::string sensor_ip_;     /**< IP-Adress of Safety laser scanner */
  std::string frame_id_;      /**< ROS Frame ID */
  double angle_start_;        /**< Start angle of measurement in radian */
  double angle_end_;          /**< End angle of measurement in radian */
  double x_axis_rotation_;    /**< Rotation of x-axis arround the center in radian */
};
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ROS_PARAMETER_HANDLER_H
