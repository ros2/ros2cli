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

from ros2cli.node import HIDDEN_NODE_PREFIX, CLI_NODE_NAME_PREFIX
DEFAULT_TIMEOUT = 0.5


class DirectNode:

    def __init__(self, args):
        timeout_reached = False

        def timer_callback():
            nonlocal timeout_reached
            timeout_reached = True

        rclpy.init()

        node_name_suffix = getattr(
            args, 'node_name_suffix', '_%d' % os.getpid())
        node_name_prefix = HIDDEN_NODE_PREFIX + CLI_NODE_NAME_PREFIX
        self.node = rclpy.create_node(node_name_prefix + node_name_suffix)
        timeout = getattr(args, 'spin_time', DEFAULT_TIMEOUT)
        timer = self.node.create_timer(timeout, timer_callback)

        while not timeout_reached:
            rclpy.spin_once(self.node)

        self.node.destroy_timer(timer)

    def __enter__(self):
        return self

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
        help='Spin time to wait for discovery (in seconds)')
