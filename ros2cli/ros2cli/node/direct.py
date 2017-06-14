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

import rclpy

# TODO(mikaelarguedas) revisit this once it's specified
HIDDEN_NODE_PREFIX = '_'

DEFAULT_TIMEOUT = 0.5


class DirectNode(object):

    def __init__(self, args):
        timeout_reached = False

        def timer_callback():
            nonlocal timeout_reached
            timeout_reached = True

        rclpy.init()

        self.node = rclpy.create_node(HIDDEN_NODE_PREFIX + 'ros2cli_node')
        timeout = getattr(args, 'spin_time', DEFAULT_TIMEOUT)
        self.timer = self.node.create_timer(timeout, timer_callback)

        while not timeout_reached:
            rclpy.spin_once(self.node)

    def __enter__(self):
        return self

    def __getattr__(self, name):
        if not rclpy.ok():
            raise RuntimeError('!rclpy.ok()')

        return getattr(self.node, name)

    def __exit__(self, exc_type, exc_value, traceback):
        self.node.destroy_timer(self.timer)
        self.node.destroy_node()
        rclpy.shutdown()


def add_arguments(parser):
    parser.add_argument(
        '--spin-time', type=float, default=DEFAULT_TIMEOUT,
        help='Spin time to wait for discovery (in seconds)')
