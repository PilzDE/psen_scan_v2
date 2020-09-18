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

#ifndef PSEN_SCAN_V2_ROS_SCANNER_NODE_H
#define PSEN_SCAN_V2_ROS_SCANNER_NODE_H

#include <stdexcept>
#include <string>
#include <atomic>
#include <gtest/gtest_prod.h>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "psen_scan_v2/scanner_data.h"

#include "psen_scan_v2/scanner.h"

namespace psen_scan_v2
{
/**
 * @brief ROS Node for fetching and publishing laserscan data from the scanner.
 *
 */
template <typename S = Scanner>
class ROSScannerNodeT
{
public:
  /**
   * @brief Constructor.
   *
   * @param nh Node handle for the ROS node on which the scanner topic is advertised.
   * @param topic Name of the ROS topic under which the scanner data are published.
   * @param frame_id Name of the frame id.
   * @param x_axis_rotation Rotation of 2D scan around the z-axis.
   * @param scanner_config Scanner configuration.
   */
  ROSScannerNodeT(ros::NodeHandle& nh,
                  const std::string& topic,
                  const std::string& frame_id,
                  const double& x_axis_rotation,
                  const ScannerConfiguration& scanner_config);

  //! @brief Continuously fetches data from the scanner and publishes the data as ROS scanner message.
  void run();
  //! @brief Terminates the fetching and publishing of scanner data.
  void terminate();

private:
  sensor_msgs::LaserScan toRosMessage(const LaserScan& laserscan) const;

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  std::string frame_id_;
  double x_axis_rotation_;
  S scanner_;
  std::atomic_bool terminate_{ false };

  friend class RosScannerNodeTests;
  FRIEND_TEST(RosScannerNodeTests, testScannerInvocation);
  FRIEND_TEST(RosScannerNodeTests, testScanTopicReceived);
  FRIEND_TEST(RosScannerNodeTests, testScanBuildFailure);
};

typedef ROSScannerNodeT<> ROSScannerNode;

template <typename S>
ROSScannerNodeT<S>::ROSScannerNodeT(ros::NodeHandle& nh,
                                    const std::string& topic,
                                    const std::string& frame_id,
                                    const double& x_axis_rotation,
                                    const ScannerConfiguration& scanner_config)
  : nh_(nh), frame_id_(frame_id), x_axis_rotation_(x_axis_rotation), scanner_(scanner_config)
{
  pub_ = nh_.advertise<sensor_msgs::LaserScan>(topic, 1);
}

template <typename S>
sensor_msgs::LaserScan ROSScannerNodeT<S>::toRosMessage(const LaserScan& laserscan) const
{
  if (!laserscan.isValid())
  {
    throw std::invalid_argument("Calculated number of measures doesn't match actual number of measures.");
  }

  sensor_msgs::LaserScan ros_message;
  ros_message.header.stamp = ros::Time::now();
  ros_message.header.frame_id = frame_id_;
  ros_message.angle_min = laserscan.getMinScanAngle() - x_axis_rotation_;
  ros_message.angle_max = laserscan.getMaxScanAngle() - x_axis_rotation_;
  ros_message.angle_increment = laserscan.getScanResolution();
  ros_message.time_increment = SCAN_TIME / NUMBER_OF_SAMPLES_FULL_SCAN_MASTER;
  ros_message.scan_time = SCAN_TIME;
  ros_message.range_min = 0;
  ros_message.range_max = 10;
  ros_message.ranges.insert(ros_message.ranges.end(),
                            laserscan.getMeasurements().crbegin(),
                            laserscan.getMeasurements().crend());  // reverse order
  std::transform(ros_message.ranges.begin(), ros_message.ranges.end(), ros_message.ranges.begin(), [](float f) {
    return f * 0.001;
  });

  return ros_message;
}

template <typename S>
void ROSScannerNodeT<S>::terminate()
{
  terminate_ = true;
}

template <typename S>
void ROSScannerNodeT<S>::run()
{
  ros::Rate r(10);
  scanner_.start();
  while (ros::ok() && !terminate_)
  {
    try
    {
      pub_.publish(toRosMessage(scanner_.getCompleteScan()));
    }
    catch (const LaserScanBuildFailure& ex)
    {
      ROS_ERROR_STREAM(ex.what());
    }
    r.sleep();
  }
  scanner_.stop();
}

}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_ROS_SCANNER_NODE_H
