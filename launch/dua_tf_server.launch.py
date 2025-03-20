"""
DUA TF Server app launch file.

dotX Automation <info@dotxautomation.com>

February 14, 2025
"""

"""
Copyright 2025 dotX Automation s.r.l.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http: // www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""


from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Builds a LaunchDescription for the DUA TF Server app."""
    ld = LaunchDescription()

    # Get config files
    config = os.path.join(
        get_package_share_directory('dua_tf_server'),
        'config',
        'dua_tf_server.yaml'
    )

    # Declare launch arguments
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='')
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config)
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_launch_arg)

    # Create node launch description
    node = Node(
        package='dua_tf_server',
        executable='dua_tf_server_app',
        namespace=ns,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[cf],
        arguments=['--ros-args', '--log-level', 'info']
    )
    ld.add_action(node)

    return ld
