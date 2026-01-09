#!/usr/bin/env python3
# Copyright 2025 Tomoya Fujita, Fumiya Ohnishi
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

"""Talker node fixture that publishes log messages at different levels."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    """Test node that publishes messages and generates logs."""

    def __init__(self):
        super().__init__('talker', enable_logger_service=True)
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0
        self.get_logger().info('Talker node started')

    def timer_callback(self):
        """Publish messages and generate various log levels."""
        msg = String()
        msg.data = f'Publishing: Hello World {self.count}'
        self.publisher_.publish(msg)

        # Generate different log levels for testing
        self.get_logger().debug(f'Debug message {msg.data}')
        self.get_logger().info(f'Info message: {msg.data}')
        self.get_logger().warning(f'Warning message {msg.data}')
        self.get_logger().error(f'Error message {msg.data}')

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
