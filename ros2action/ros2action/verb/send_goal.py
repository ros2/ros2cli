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

from collections import OrderedDict
import importlib
import sys
import yaml

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from ros2action.verb import VerbExtension
from ros2cli.node import NODE_NAME_PREFIX
# TODO(jacobperron): Move 'set_msg_fields' to ros2cli package to remove dependency to ros2topic
from ros2topic.api import set_msg_fields
from ros2topic.api import SetFieldError


class SendGoalVerb(VerbExtension):
    """Send an action goal."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'action_name',
            help="Name of the ROS action (e.g. '/fibonacci')")
        parser.add_argument(
            'action_type',
            help="Type of the ROS action (e.g. 'example_interfaces/Fibonacci')")
        parser.add_argument(
            'goal',
            help="Goal request values in YAML format (e.g. '{order: 10}')")
        parser.add_argument(
            '-f', '--feedback', action='store_true',
            help='Echo feedback messages for the goal')

    def main(self, *, args):
        register_yaml_representer()
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

# TODO(jacobperron): Move to common place in ros2cli for all packages to use
def register_yaml_representer():
    # Register our custom representer for YAML output
    yaml.add_representer(OrderedDict, represent_ordereddict)


# TODO(jacobperron): Move to common place in ros2cli for all packages to use
# Custom representer for getting clean YAML output that preserves the order in
# an OrderedDict.
# Inspired by:
# http://stackoverflow.com/a/16782282/7169408
def represent_ordereddict(dumper, data):
    items = []
    for k, v in data.items():
        items.append((dumper.represent_data(k), dumper.represent_data(v)))
    return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', items)


# TODO(jacobperron): Move to common place in ros2cli for all packages to use
def _convert_value(value, truncate_length=None):
    if isinstance(value, bytes):
        if truncate_length is not None and len(value) > truncate_length:
            value = ''.join([chr(c) for c in value[:truncate_length]]) + '...'
        else:
            value = ''.join([chr(c) for c in value])
    elif isinstance(value, str):
        if truncate_length is not None and len(value) > truncate_length:
            value = value[:truncate_length] + '...'
    elif isinstance(value, tuple) or isinstance(value, list):
        if truncate_length is not None and len(value) > truncate_length:
            # Truncate the sequence
            value = value[:truncate_length]
            # Truncate every item in the sequence
            value = type(value)([_convert_value(v, truncate_length) for v in value] + ['...'])
        else:
            # Truncate every item in the list
            value = type(value)([_convert_value(v, truncate_length) for v in value])
    elif isinstance(value, dict) or isinstance(value, OrderedDict):
        # convert each key and value in the mapping
        new_value = {} if isinstance(value, dict) else OrderedDict()
        for k, v in value.items():
            # don't truncate keys because that could result in key collisions and data loss
            new_value[_convert_value(k)] = _convert_value(v, truncate_length=truncate_length)
        value = new_value
    elif not any(isinstance(value, t) for t in (bool, float, int)):
        # assuming value is a message
        # since it is neither a collection nor a primitive type
        value = msg_to_ordereddict(value, truncate_length=truncate_length)
    return value


# TODO(jacobperron): Move to common place in ros2cli for all packages to use
def msg_to_ordereddict(msg, truncate_length=None):
    d = OrderedDict()
    # We rely on __slots__ retaining the order of the fields in the .msg file.
    for field_name in msg.__slots__:
        value = getattr(msg, field_name, None)
        value = _convert_value(value, truncate_length=truncate_length)
        # remove leading underscore from field name
        d[field_name[1:]] = value
    return d


# TODO(jacobperron): Move to common place in ros2cli for all packages to use
def msg_to_yaml(msg):
    return yaml.dump(
        msg_to_ordereddict(
          msg,
          truncate_length=128
          # truncate_length=args.truncate_length if not args.full_length else None
        ), width=sys.maxsize)


def _feedback_callback(feedback):
    print('Feedback:\n    {}'.format(msg_to_yaml(feedback.feedback)))
    # print(msg_to_yaml(feedback.feedback))


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
        set_msg_fields(goal, goal_dict)
    except SetFieldError as ex:
        return "Failed to populate field '{ex.field_name}': {ex.exception}" \
            .format_map(locals())

    print('Waiting for an action server to become available...')
    action_client.wait_for_server()

    print('Sending goal:\n     {}'.format(msg_to_yaml(goal)))
    # print(msg_to_yaml(goal))
    goal_future = action_client.send_goal_async(goal, feedback_callback)
    rclpy.spin_until_future_complete(node, goal_future)

    goal_handle = goal_future.result()

    if goal_handle is None:
        raise RuntimeError('Exeception while sending goal: {!r}'.format(goal_future.exception()))

    if not goal_handle.accepted:
        print('Goal was rejected.')
        return

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result()

    if result is None:
        raise RuntimeError(
            'Exeception while getting result: {!r}'.format(result_future.exception()))

    print('Result:\n    {}'.format(msg_to_yaml(result.result)))
    # print(msg_to_yaml(result.result))
    print('Goal finished with status: {}'.format(_goal_status_to_string(result.status)))

    action_client.destroy()
    node.destroy_node()
    rclpy.shutdown()
