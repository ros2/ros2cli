# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import sys

from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage

import yaml


def get_talker_node_action(*, node_name='my_talker',
                           node_namespace='my_ns',
                           topic_name='chatter'):
    return Node(
        package='demo_nodes_py', node_executable='talker', node_name=node_name, name=node_name,
        node_namespace=node_namespace, remappings=[('chatter', topic_name)], output='screen',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )


def get_publisher_node_action(*, topic_name='/my_ns/test', topic_type='test_msgs/msg/BasicTypes'):
    return ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', topic_name, topic_type],
        name='publisher', output='screen',
        sigterm_timeout=LaunchConfiguration(
            'sigterm_timeout', default=60
        )
    )


def get_dummy_base_controller_node_action(*, topic_name='/my_ns/cmd_vel', start_time=0.0):
    return ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub',
            '-r', '1', topic_name,
            'geometry_msgs/msg/TwistStamped',
            yaml.dump({
                'header': {
                    'stamp': {
                        'sec': int(start_time),
                        'nanosec': int((start_time - int(start_time)) * 1e9)
                    }
                }
            }, default_flow_style=True).replace('\n', '')
        ], name='dummy_base_controller', output='screen',
        sigterm_timeout=LaunchConfiguration(
            'sigterm_timeout', default=60
        )
    )


def get_listener_node_action(*, node_name='my_listener',
                             node_namespace='my_ns',
                             topic_name='chatter'):
    return Node(
        package='demo_nodes_py', node_executable='listener', node_name=node_name, name=node_name,
        node_namespace=node_namespace, remappings=[('chatter', topic_name)], output='screen',
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )


def get_add_two_ints_server_action(*, node_name='my_add_two_ints_server',
                                   node_namespace='my_ns',
                                   service_name='add_two_ints'):
    return Node(
        package='demo_nodes_py', node_executable='add_two_ints_server',
        node_name=node_name, node_namespace=node_namespace,
        output='screen', remappings=[('add_two_ints', service_name)],
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )


def get_fibonacci_action_server_node_action():
    return ExecuteProcess(
        cmd=[
            sys.executable,
            ExecutableInPackage('fibonacci_action_server.py', 'action_tutorials'),
        ],
        sigterm_timeout=LaunchConfiguration('sigterm_timeout', default=30)
    )
