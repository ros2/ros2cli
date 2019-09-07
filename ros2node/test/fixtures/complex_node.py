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

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from test_msgs.action import Fibonacci
from test_msgs.msg import Arrays
from test_msgs.msg import Strings
from test_msgs.srv import BasicTypes


class ComplexNode(Node):

    def __init__(self):
        super().__init__('complex_node')
        self.publisher = self.create_publisher(Arrays, 'arrays', qos_profile_system_default)
        self.subscription = self.create_subscription(
            Strings, 'strings', lambda msg: None, qos_profile_system_default
        )
        self.server = self.create_service(BasicTypes, 'basic', lambda req, res: res)
        self.action_server = ActionServer(
            self, Fibonacci, 'fibonacci', self.action_callback
        )
        self.timer = self.create_timer(1.0, self.pub_callback)

    def destroy_node(self):
        self.timer.destroy()
        self.publisher.destroy()
        self.subscription.destroy()
        self.server.destroy()
        self.action_server.destroy()
        super().destroy_node()

    def pub_callback(self):
        self.publisher.publish(Arrays())

    def action_callback(self, goal_handle):
        goal_handle.succeed()
        return Fibonacci.Result()


def main(args=None):
    rclpy.init(args=args)

    node = ComplexNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('node stopped cleanly')
    except BaseException:
        print('exception in node:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
