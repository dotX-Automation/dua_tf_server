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
    cf = LaunchConfiguration('cf')
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config)
    ld.add_action(cf_launch_arg)

    # Launch the DUA TF Server container
    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='dua_app_management',
        executable='dua_component_container_mt',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        arguments=['--ros-args', '--log-level', 'info'],
        composable_node_descriptions=[
            # DUA tf server node
            ComposableNode(
                package='dua_tf_server',
                plugin='dua_tf_server::TFServerNode',
                name='dua_tf_server',
                namespace='',
                parameters=[cf],
                extra_arguments=[{'use_intra_process_comms': True}]),
        ],
    )
    ld.add_action(container)

    return ld
