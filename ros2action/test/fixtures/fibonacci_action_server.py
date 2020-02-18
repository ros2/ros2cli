#!/usr/bin/env python3
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
# limitations under the License.import time

import sys

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from test_msgs.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()

    def execute_callback(self, goal_handle):
        feedback = Fibonacci.Feedback()
        feedback.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback.sequence.append(feedback.sequence[i] + feedback.sequence[i-1])
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback.sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    node = FibonacciActionServer()
    try:
        # Spend up to 2 seconds doing all the work we need to have the fibonacci
        # action server actually talking over DDS before signaling to the test it's
        # OK to start
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
        print('fibonacci_action_server has started')
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('server stopped cleanly')
    except BaseException:
        print('exception in server:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
