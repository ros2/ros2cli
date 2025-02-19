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

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from test_msgs.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server', namespace='/test')
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
    try:
        with rclpy.init(args=args):
            node = FibonacciActionServer()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('server stopped cleanly')


if __name__ == '__main__':
    main()
