# Copyright (c) 2021 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import asyncio
import os
import pytest
import unittest

from math import radians

import launch
import launch_testing

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def get_optional_env(name, default_value):
    return os.environ[name] if name in os.environ else default_value


@pytest.mark.launch_test
def generate_test_description():
    bringup_launch_descr = XMLLaunchDescriptionSource(
        PathJoinSubstitution([
            get_package_share_directory('psen_scan_v2'),
            'launch',
            'bringup.launch.xml'
        ])
    )

    sensor_ip = get_optional_env('SENSOR_IP', '192.168.0.10')
    host_ip = get_optional_env('HOST_IP', 'auto')
    bringup_launch_args = {
        'sensor_ip': sensor_ip,
        'host_ip': host_ip,
        'host_udp_port_data': LaunchConfiguration('host_udp_port_data'),
        'angle_start': LaunchConfiguration('angle_start'),
        'angle_end':  LaunchConfiguration('angle_end'),
        'resolution': LaunchConfiguration('resolution'),
        'intensities': LaunchConfiguration('intensities')
    }

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='host_udp_port_data'),
        DeclareLaunchArgument(name='angle_start', default_value=str(radians(-137.4))),
        DeclareLaunchArgument(name='angle_end', default_value=str(radians(137.4))),
        DeclareLaunchArgument(name='resolution', default_value=str(radians(0.1))),
        DeclareLaunchArgument(name='intensities', default_value='False'),
        IncludeLaunchDescription(bringup_launch_descr,
                                 launch_arguments=bringup_launch_args.items()),
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ])


class LaserscanSubscriber(Node):
    """ Subscriber returning the 5-th single message via future.
        (Avoid testing a message with an old config which can happen at scanner start)
    """

    def __init__(self, future):
        super().__init__('laserscan_subscriber')

        self.future = future
        self.message_count = 0
        self.subscription = self.create_subscription(LaserScan, 'laser_1/scan', self.callback, 5)

    def callback(self, msg):
        self.message_count += 1
        if self.message_count > 4 and not self.future.done():
            self.future.set_result(msg)


async def get_single_laserscan_msg() -> LaserScan:
    loop = asyncio.get_running_loop()
    subscriber_future = loop.create_future()
    subscriber = LaserscanSubscriber(subscriber_future)

    rclpy.spin_until_future_complete(subscriber, subscriber_future, timeout_sec=10.0)
    return subscriber_future.result()


def to_multiple_of_tenth_degree_in_radian(radian_angle: float) -> float:
    reminder = radian_angle % radians(0.1)
    if (reminder < 0.5 * radians(0.1)):
        return radian_angle - reminder
    return radian_angle + reminder


DECIMAL_PLACE_ACCURACY = 6


class HwtestScanRange(unittest.IsolatedAsyncioTestCase):

    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def read_parameters_from_context(self, context):
        self.angle_start = float(context.launch_configurations['angle_start'])
        self.angle_end = float(context.launch_configurations['angle_end'])
        self.resolution = float(context.launch_configurations['resolution'])

    async def test_scan_range(self, launch_service):
        try:
            self.read_parameters_from_context(launch_service.context)
        except KeyError:
            self.fail("Failure reading parameters from launch context")

        message: LaserScan = await get_single_laserscan_msg()

        angle_start_rounded = to_multiple_of_tenth_degree_in_radian(self.angle_start)
        angle_end_rounded = to_multiple_of_tenth_degree_in_radian(self.angle_end)
        resolution_rounded = to_multiple_of_tenth_degree_in_radian(self.resolution)

        expected_max_angle = angle_end_rounded - \
            ((angle_end_rounded - angle_start_rounded) % resolution_rounded)

        self.assertAlmostEqual(angle_start_rounded, message.angle_min, DECIMAL_PLACE_ACCURACY,
                               "angle_min of the laserscan message is " + str(message.angle_min) +
                               " but should be " + str(angle_start_rounded) + ".")
        self.assertAlmostEqual(expected_max_angle, message.angle_max, DECIMAL_PLACE_ACCURACY,
                               "angle_max of the laserscan message is " + str(message.angle_max) +
                               " but should be " + str(expected_max_angle) + ".")


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
