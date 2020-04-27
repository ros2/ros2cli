# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import rclpy
import rclpy.action

from ros2cli.node import NODE_NAME_PREFIX
DEFAULT_TIMEOUT = 0.5


class DirectNode:

    def __init__(self, args, *, node_name=None):
        timeout_reached = False

        def timer_callback():
            nonlocal timeout_reached
            timeout_reached = True

        argv = getattr(args, 'argv', [])

        rclpy.init(args=argv)

        node_name_suffix = getattr(
            args, 'node_name_suffix', '_%d' % os.getpid())
        start_parameter_services = getattr(
            args, 'start_parameter_services', False)

        if node_name is None:
            node_name = NODE_NAME_PREFIX + node_name_suffix

        self.node = rclpy.create_node(
            node_name,
            start_parameter_services=start_parameter_services)
        timeout = getattr(args, 'spin_time', DEFAULT_TIMEOUT)
        timer = self.node.create_timer(timeout, timer_callback)

        while not timeout_reached:
            rclpy.spin_once(self.node)

        self.node.destroy_timer(timer)

    def __enter__(self):
        return self

    # TODO(hidmic): generalize/standardize rclpy graph API
    #               to not have to make a special case for
    #               rclpy.action
    def get_action_names_and_types(self):
        return rclpy.action.get_action_names_and_types(self.node)

    def get_action_client_names_and_types_by_node(self, remote_node_name, remote_node_namespace):
        return rclpy.action.get_action_client_names_and_types_by_node(
            self.node, remote_node_name, remote_node_namespace)

    def get_action_server_names_and_types_by_node(self, remote_node_name, remote_node_namespace):
        return rclpy.action.get_action_server_names_and_types_by_node(
            self.node, remote_node_name, remote_node_namespace)

    def __getattr__(self, name):
        if not rclpy.ok():
            raise RuntimeError('!rclpy.ok()')

        return getattr(self.node, name)

    def __exit__(self, exc_type, exc_value, traceback):
        self.node.destroy_node()
        rclpy.shutdown()


def add_arguments(parser):
    parser.add_argument(
        '--spin-time', type=float, default=DEFAULT_TIMEOUT,
        help='Spin time in seconds to wait for discovery (only applies when '
             'not using an already running daemon)')
