#!/usr/bin/env python3
# Copyright 2026 Tomoya Fujita, Fumiya Ohnishi
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

"""Listener node fixture that subscribes and generates logs."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    """Test node that subscribes to messages and generates logs."""

    def __init__(self):
        super().__init__('listener', enable_logger_service=False)
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.get_logger().info('Listener node started')

    def listener_callback(self, msg: String) -> None:
        """Process received messages and log them."""
        self.get_logger().debug(f'Debug message: {msg.data}')
        self.get_logger().info(f'Info message: {msg.data}')
        self.get_logger().warning(f'Warning message: {msg.data}')
        self.get_logger().error(f'Error message: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
