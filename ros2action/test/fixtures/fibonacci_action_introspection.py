# Copyright 2025 Sony Group Corporation.
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
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState

from test_msgs.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server', namespace='/test')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)
        self._action_server.configure_introspection(
            self.get_clock(), qos_profile_system_default, ServiceIntrospectionState.CONTENTS)

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


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client', namespace='/test')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self._action_client.configure_introspection(
            self.get_clock(), qos_profile_system_default, ServiceIntrospectionState.CONTENTS)
        self._timer = self.create_timer(3, self._timer_callback)
        self._send_goal_future = None

    def _timer_callback(self):
        if not self._action_client.wait_for_server():
            self.get_logger().info('Action server is unavailable.')
            return

        if self._send_goal_future is None:
            self.send_goal(2)
            return

        if not self._send_goal_future.done():
            return

        if self._send_goal_future.result() is None:
            self.get_logger().error('Exception generated on the goal: {0}'.format(
                self._send_goal_future.exception()))

        self._send_goal_future = None

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.sequence))

    def destroy_node(self):
        self._action_client.destroy()
        super().destroy_node()


def main(args=None):
    try:
        with rclpy.init(args=args):
            action_server_node = FibonacciActionServer()
            action_client_node = FibonacciActionClient()

            executor = SingleThreadedExecutor()
            executor.add_node(action_server_node)
            executor.add_node(action_client_node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        print('server stopped cleanly')


if __name__ == '__main__':
    main()
