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
import unittest

import launch
import launch_ros
import launch_testing

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

import pytest

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@pytest.mark.launch_test
def generate_test_description():
    bringup_launch_descr = XMLLaunchDescriptionSource(
        PathJoinSubstitution([
            get_package_share_directory('psen_scan_v2'),
            'launch',
            'bringup.launch.xml'
        ])
    )

    bringup_launch_args = {'angle_start': '-1.2', 'angle_end': '1.2',
                           'sensor_ip': '192.168.0.10', 'host_ip': 'auto', 'host_udp_port_data': '55000'}

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            bringup_launch_descr,
            launch_arguments=bringup_launch_args.items()
        ),
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ])


class LaserscanSubscriber(Node):
    """Subscriber marking a future as done once it received a single message."""

    def __init__(self, future):
        super().__init__('laserscan_subscriber')

        self.future = future
        self.subscription = self.create_subscription(LaserScan, 'laser_1/scan', self.callback, 1)

    def callback(self, msg):
        self.future.set_result('laserscan msg received')


class TestNodeAvailable(unittest.IsolatedAsyncioTestCase):

    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    async def test_scan_topic_available(self):
        loop = asyncio.get_running_loop()
        subscriber_future = loop.create_future()
        subscriber_node = LaserscanSubscriber(subscriber_future)

        rclpy.spin_until_future_complete(subscriber_node, subscriber_future, timeout_sec=10.0)
        self.assertTrue(subscriber_future.done())


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
