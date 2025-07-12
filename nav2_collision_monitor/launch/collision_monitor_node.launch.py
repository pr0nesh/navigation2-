#!/usr/bin/env python3

# Copyright (c) 2022 Samsung R&D Institute Russia
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

if os.getenv('ROS_DISTRO') == 'humble':
    from launch_ros.actions import PushRosNamespace
else:
    from launch.substitutions import NotEqualsSubstitution
    from launch_ros.actions import PushROSNamespace

from launch_ros.actions import LoadComposableNodes, SetParameter, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Environment
    package_dir = get_package_share_directory('nav2_collision_monitor')

    # Constant parameters
    lifecycle_nodes = ['collision_monitor']
    autostart = True
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    container_name = LaunchConfiguration('container_name')
    use_composition = LaunchConfiguration('use_composition')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', 
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(package_dir, 'params', 'collision_monitor_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )
    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='collision_monitor_container',
        description='Container name for collision monitor nodes'
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup'
    )

    # Create parameter substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file, 
            root_key=namespace,
            param_rewrites=param_substitutions, 
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Create container name with namespace
    container_name_full = PathJoinSubstitution([namespace, container_name])

    # Namespace action
    if os.getenv('ROS_DISTRO') == 'humble':
        namespace_action = PushRosNamespace(
            condition=IfCondition(PythonExpression(['"" != "', namespace, '"'])),
            namespace=namespace,
        )
    else:
        namespace_action = PushROSNamespace(
            condition=IfCondition(NotEqualsSubstitution(namespace, '')),
            namespace=namespace,
        )

    # Create the container
    container_node = Node(
        condition=IfCondition(use_composition),
        name=container_name,
        package='rclcpp_components',
        executable='component_container',
        parameters=[configured_params],
        remappings=remappings,
        output='screen',
    )

    # Load composable nodes
    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            namespace_action,
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_collision_monitor',
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                ],
            ),
        ],
    )

    # Non-composed nodes (alternative)
    start_lifecycle_manager_cmd = Node(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ],
        namespace=namespace,
        remappings=remappings
    )

    start_collision_monitor_cmd = Node(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[configured_params],
        namespace=namespace,
        remappings=remappings
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_composition_cmd)

    # Add nodes
    ld.add_action(container_node)
    ld.add_action(load_composable_nodes)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_collision_monitor_cmd)

    return ld
