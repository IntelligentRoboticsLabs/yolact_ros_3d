# Copyright 2020 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

params_file1 = '/config/yolact_3d.yaml'
params_file2 = '/config/octomaps_config.yaml'


def generate_launch_description():

    # Load params:

    pkg_dir = get_package_share_directory('yolact_ros2_3d')
    config_file1_path = pkg_dir + params_file1
    config_file2_path = pkg_dir + params_file2

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create Node:

    yolact3d_node = Node(
        package='yolact_ros2_3d',
        node_executable='yolact_ros2_3d_node',
        node_name='yolact_ros2_3d_node',
        output='screen',
        parameters=[config_file1_path, config_file2_path],
        remappings=[
            ('/yolact_ros2_3d/octomaps/dynamics/person', '/dummy_octomap')
        ]
    )

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(yolact3d_node)

    return ld
