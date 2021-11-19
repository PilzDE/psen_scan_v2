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

import os
import pytest
import unittest

import launch
import launch_ros
import launch_testing

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


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
        'angle_start': '-1.2',
        'angle_end': '1.2',
        'host_udp_port_data': '55005'
    }

    test_node = launch_ros.actions.Node(
        executable=PathJoinSubstitution(
            [LaunchConfiguration('test_binary_dir'), 'hwtest_scan_compare']),
        parameters=[{'test_duration': LaunchConfiguration('test_duration')}],
        output='screen'
    )

    return launch.LaunchDescription([
        DeclareLaunchArgument(name='test_binary_dir',
                              description='Binary directory of package containing test executables'),
        DeclareLaunchArgument(name='test_duration',
                              description='Duration of test',
                              default_value='10'),
        IncludeLaunchDescription(
            bringup_launch_descr,
            launch_arguments=bringup_launch_args.items()
        ),
        test_node,
        launch_testing.util.KeepAliveProc(),
        launch_testing.actions.ReadyToTest(),
    ]), {'scan_compare_gtest': test_node}


class TestScanCompareGTest(unittest.TestCase):

    # This prevents the launch-test to shutdown too early
    def test_gtest_terminates(self, launch_service, proc_info, scan_compare_gtest):
        try:
            test_duration = int(launch_service.context.launch_configurations['test_duration'])
        except KeyError:
            self.fail("Failure reading parameters from launch context")
        proc_info.assertWaitForShutdown(process=scan_compare_gtest, timeout=test_duration + 5)


@launch_testing.post_shutdown_test()
class TestOutcome(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
