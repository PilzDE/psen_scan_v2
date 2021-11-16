# Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

import time
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


@pytest.mark.launch_test
def generate_test_description():
    psen_scan_v2_launch_descr = XMLLaunchDescriptionSource(
        PathJoinSubstitution([
            get_package_share_directory('psen_scan_v2'),
            'launch',
            'psen_scan_v2.launch.xml'
        ])
    )

    psen_scan_v2_launch_args = {'sensor_ip': '127.0.0.1', 'rviz': 'false'}

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            psen_scan_v2_launch_descr,
            launch_arguments=psen_scan_v2_launch_args.items()
        ),
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ])


class TestNodeAvailable(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('TestNodeAvailable')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_psen_scan_v2_node_available(self):
        node_found = False
        end_time = time.time() + 10
        while time.time() < end_time:
            if 'laser_1' in self.node.get_node_names():
                node_found = True
                break
        self.assertTrue(node_found)

    def test_robot_state_publisher_node_available(self):
        node_found = False
        end_time = time.time() + 10
        while time.time() < end_time:
            if 'robot_state_publisher' in self.node.get_node_names():
                node_found = True
                break
        self.assertTrue(node_found)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
