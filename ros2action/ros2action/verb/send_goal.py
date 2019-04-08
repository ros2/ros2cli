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

import importlib

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from ros2action.api import action_name_completer
from ros2action.api import ActionTypeCompleter
from ros2action.verb import VerbExtension
from ros2cli.node import NODE_NAME_PREFIX
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py import set_message_fields

import yaml


class SendGoalVerb(VerbExtension):
    """Send an action goal."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'action_name',
            help="Name of the ROS action (e.g. '/fibonacci')")
        arg.completer = action_name_completer
        arg = parser.add_argument(
            'action_type',
            help="Type of the ROS action (e.g. 'example_interfaces/Fibonacci')")
        arg.completer = ActionTypeCompleter(action_name_key='action_name')
        parser.add_argument(
            'goal',
            help="Goal request values in YAML format (e.g. '{order: 10}')")
        parser.add_argument(
            '-f', '--feedback', action='store_true',
            help='Echo feedback messages for the goal')

    def main(self, *, args):
        feedback_callback = None
        if args.feedback:
            feedback_callback = _feedback_callback
        return send_goal(args.action_name, args.action_type, args.goal, feedback_callback)


def _goal_status_to_string(status):
    if GoalStatus.STATUS_ACCEPTED == status:
        return 'ACCEPTED'
    elif GoalStatus.STATUS_EXECUTING == status:
        return 'EXECUTING'
    elif GoalStatus.STATUS_CANCELING == status:
        return 'CANCELING'
    elif GoalStatus.STATUS_SUCCEEDED == status:
        return 'SUCCEEDED'
    elif GoalStatus.STATUS_CANCELED == status:
        return 'CANCELED'
    elif GoalStatus.STATUS_ABORTED == status:
        return 'ABORTED'
    else:
        return 'UNKNOWN'


def _feedback_callback(feedback):
    print('Feedback:\n    {}'.format(message_to_yaml(feedback.feedback, None)))


def send_goal(action_name, action_type, goal_values, feedback_callback):
    # TODO(jacobperron): This logic should come from a rosidl related package
    package_name, action_type = action_type.split('/', 2)
    if not package_name or not action_type:
        raise RuntimeError('The passed action type is invalid')

    module = importlib.import_module(package_name + '.action')
    action_module = getattr(module, action_type)
    goal_dict = yaml.load(goal_values)

    rclpy.init()

    node_name = NODE_NAME_PREFIX + '_send_goal_{}_{}'.format(package_name, action_type)
    node = rclpy.create_node(node_name)

    action_client = ActionClient(node, action_module, action_name)

    goal = action_module.Goal()

    try:
        set_message_fields(goal, goal_dict)
    except Exception as ex:
        return 'Failed to populate message fields: {!r}'.format(ex)

    print('Waiting for an action server to become available...')
    action_client.wait_for_server()

    print('Sending goal:\n     {}'.format(message_to_yaml(goal, None)))
    goal_future = action_client.send_goal_async(goal, feedback_callback)
    rclpy.spin_until_future_complete(node, goal_future)

    goal_handle = goal_future.result()

    if goal_handle is None:
        raise RuntimeError('Exeception while sending goal: {!r}'.format(goal_future.exception()))

    if not goal_handle.accepted:
        print('Goal was rejected.')
        return

    print('Goal accepted with ID: {}\n'.format(bytes(goal_handle.goal_id.uuid).hex()))

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result()

    if result is None:
        raise RuntimeError(
            'Exeception while getting result: {!r}'.format(result_future.exception()))

    print('Result:\n    {}'.format(message_to_yaml(result.result, None)))
    print('Goal finished with status: {}'.format(_goal_status_to_string(result.status)))

    action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()
