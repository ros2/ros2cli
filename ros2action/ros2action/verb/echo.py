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

from collections import OrderedDict
from enum import Enum
import queue
import sys
import threading

import rclpy
from rclpy.qos import qos_profile_action_status_default
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription
from rclpy.type_support import EventMessage
from rclpy.type_support import MsgT

from ros2action.api import action_name_completer
from ros2action.api import action_type_completer
from ros2action.api import get_action_class
from ros2action.verb import VerbExtension

from ros2cli.helpers import unsigned_int
from ros2cli.node.strategy import NodeStrategy
from ros2cli.qos import add_qos_arguments
from ros2cli.qos import profile_configure_short_keys

from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_action

from service_msgs.msg import ServiceEventInfo

import yaml

DEFAULT_TRUNCATE_LENGTH = 128


class ActionInterfaces(Enum):
    GOAL_SERVICE = 'GOAL_SERVICE'
    CANCEL_SERVICE = 'CANCEL_SERVICE'
    RESULT_SERVICE = 'RESULT_SERVICE'
    FEEDBACK_TOPIC = 'FEEDBACK_TOPIC'
    STATUS_TOPIC = 'STATUS_TOPIC'


class EchoVerb(VerbExtension):
    """Echo a action."""

    # Custom representer for getting clean YAML output that preserves the order in an OrderedDict.
    # Inspired by: http://stackoverflow.com/a/16782282/7169408
    def __represent_ordereddict(self, dumper, data):
        items = []
        for k, v in data.items():
            items.append((dumper.represent_data(k), dumper.represent_data(v)))
        return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', items)

    def __init__(self):
        self._event_number_to_name = {}
        for k, v in ServiceEventInfo._Metaclass_ServiceEventInfo__constants.items():
            self._event_number_to_name[v] = k

        yaml.add_representer(OrderedDict, self.__represent_ordereddict)

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'action_name',
            help="Name of the ROS action to echo (e.g. '/fibonacci')")
        arg.completer = action_name_completer
        arg = parser.add_argument(
            'action_type', nargs='?',
            help="Type of the ROS action (e.g. 'example_interfaces/action/Fibonacci')")
        arg.completer = action_type_completer
        parser.add_argument(
            '--interfaces', '-i', type=str, metavar='interfaces_name_list',
            help='Specify a action interfaces list are output. "|" is used as a delimiter for '
                 'multiple action interfaces. Action interfaces include "GOAL_SERVICE", '
                 '"CANCEL_SERVICE", "RESULT_SERVICE", "FEEDBACK_TOPIC" and "STATUS_TOPIC". '
                 'If this option is not set, output messages from all interfaces of the action.')
        parser.add_argument(
            '--csv', action='store_true', default=False,
            help=(
                'Output all recursive fields separated by commas (e.g. for plotting).'
            ))
        parser.add_argument(
            '--full-length', '-f', action='store_true',
            help='Output all elements for arrays, bytes, and string with a '
                 "length > '--truncate-length', by default they are truncated "
                 "after '--truncate-length' elements with '...''")
        parser.add_argument(
            '--truncate-length', '-l', type=unsigned_int, default=DEFAULT_TRUNCATE_LENGTH,
            help='The length to truncate arrays, bytes, and string to '
                 '(default: %d)' % DEFAULT_TRUNCATE_LENGTH)
        parser.add_argument(
            '--no-arr', action='store_true', help="Don't print array fields of messages")
        parser.add_argument(
            '--no-str', action='store_true', help="Don't print string fields of messages")
        parser.add_argument(
            '--flow-style', action='store_true',
            help='Print collections in the block style (not available with csv format)')
        add_qos_arguments(parser, 'action introspection', 'services_default')

    def main(self, *, args):
        output_interface_list = set()
        if args.interfaces is not None:
            if args.interfaces:  # Not empty string
                # Fill in the user-specified action interface.
                interfaces = args.interfaces.split('|')
                for interface in interfaces:
                    if interface.strip().upper() in ActionInterfaces:
                        output_interface_list.add(interface.strip().upper())
                    else:
                        return f'"{interface.strip()}" is incorrect interface name.'
            else:
                return 'Input action interface name is empty.'
        else:
            # By default, output all the action interfaces.
            output_interface_list.update([interface.value for interface in ActionInterfaces])

        if args.action_type is None:
            with NodeStrategy(args) as node:
                try:
                    action_module = get_action_class(
                        node, args.action_name)
                except (AttributeError, ModuleNotFoundError, ValueError):
                    raise RuntimeError(f"The action name '{args.action_name}' is invalid")
        else:
            try:
                action_module = get_action(args.action_type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError(f"The service type '{args.action_type}' is invalid")

        if action_module is None:
            raise RuntimeError(f"Could not load the action type for '{args.action_type}'")

        self.csv = args.csv
        self.truncate_length = args.truncate_length if not args.full_length else None
        self.flow_style = args.flow_style
        self.no_arr = args.no_arr
        self.no_str = args.no_str

        qos_profile = QoSPresetProfiles.get_from_short_key(args.qos_profile)
        profile_configure_short_keys(
            qos_profile, args.qos_reliability, args.qos_durability,
            args.qos_depth, args.qos_history, args.qos_liveliness,
            args.qos_liveliness_lease_duration_seconds)

        send_goal_event_topic = args.action_name + '/_action/send_goal/_service_event'
        send_goal_event_msg_type = action_module.Impl.SendGoalService.Event

        cancel_goal_event_topic = args.action_name + '/_action/cancel_goal/_service_event'
        cancel_goal_event_msg_type = action_module.Impl.CancelGoalService.Event

        get_result_event_topic = args.action_name + '/_action/get_result/_service_event'
        get_result_event_msg_type = action_module.Impl.GetResultService.Event

        feedback_topic = args.action_name + '/_action/feedback'
        feedback_topic_type = action_module.Impl.FeedbackMessage

        status_topic = args.action_name + '/_action/status'
        status_topic_type = action_module.Impl.GoalStatusMessage

        # Queue for messages from above topic. The queue size is set to 100.
        # If the queue is full, the message will be dropped.
        self.output_msg_queue = queue.Queue(100)

        run_thread = True
        # Create a thread to output message from output_queue

        def message_handler():
            while run_thread:
                try:
                    message = self.output_msg_queue.get(block=True, timeout=0.5)
                    self.output_msg_queue.task_done()
                except queue.Empty:
                    continue
                print(message)
        output_msg_thread = threading.Thread(target=message_handler)
        output_msg_thread.start()

        with NodeStrategy(args) as node:
            send_goal_event_sub = None
            # TODO: The QoS for the service event publisher, feedback publisher and status
            # publisher can be specified by the user, so new parameters need to be added to allow
            # specifying QoS of subscription to replace the current fixed QoS.
            if ActionInterfaces.GOAL_SERVICE.value in output_interface_list:
                send_goal_event_sub: Subscription[EventMessage] = node.create_subscription(
                    send_goal_event_msg_type,
                    send_goal_event_topic,
                    self._send_goal_subscriber_callback,
                    qos_profile)

            cancel_goal_event_sub = None
            if ActionInterfaces.CANCEL_SERVICE.value in output_interface_list:
                cancel_goal_event_sub: Subscription[EventMessage] = node.create_subscription(
                    cancel_goal_event_msg_type,
                    cancel_goal_event_topic,
                    self._cancel_goal_subscriber_callback,
                    qos_profile)

            get_result_event_sub = None
            if ActionInterfaces.RESULT_SERVICE.value in output_interface_list:
                get_result_event_sub: Subscription[EventMessage] = node.create_subscription(
                    get_result_event_msg_type,
                    get_result_event_topic,
                    self._get_result_subscriber_callback,
                    qos_profile)

            feedback_sub = None
            if ActionInterfaces.FEEDBACK_TOPIC.value in output_interface_list:
                feedback_sub: Subscription[MsgT] = node.create_subscription(
                    feedback_topic_type,
                    feedback_topic,
                    self._feedback_subscriber_callback,
                    QoSProfile(depth=10))   # QoS setting refers to action client implementation

            status_sub = None
            if ActionInterfaces.STATUS_TOPIC.value in output_interface_list:
                status_sub: Subscription[MsgT] = node.create_subscription(
                    status_topic_type,
                    status_topic,
                    self._status_subscriber_callback,
                    qos_profile_action_status_default)

            executor: rclpy.Executor = rclpy.get_global_executor()
            try:
                executor.add_node(node)
                while executor.context.ok():
                    executor.spin_once()
            except KeyboardInterrupt:
                pass
            finally:
                executor.remove_node(node)

            if send_goal_event_sub:
                send_goal_event_sub.destroy()
            if cancel_goal_event_sub:
                cancel_goal_event_sub.destroy()
            if get_result_event_sub:
                get_result_event_sub.destroy()
            if feedback_sub:
                feedback_sub.destroy()
            if status_sub:
                status_sub.destroy()

            run_thread = False
            if output_msg_thread.is_alive():
                output_msg_thread.join(1)

    def _send_goal_subscriber_callback(self, msg):
        self._base_subscriber_callback(msg, ActionInterfaces.GOAL_SERVICE.value)

    def _cancel_goal_subscriber_callback(self, msg):
        self._base_subscriber_callback(msg, ActionInterfaces.CANCEL_SERVICE.value)

    def _get_result_subscriber_callback(self, msg):
        self._base_subscriber_callback(msg, ActionInterfaces.RESULT_SERVICE.value)

    def _feedback_subscriber_callback(self, msg):
        self._base_subscriber_callback(msg, ActionInterfaces.FEEDBACK_TOPIC.value)

    def _status_subscriber_callback(self, msg):
        self._base_subscriber_callback(msg, ActionInterfaces.STATUS_TOPIC.value)

    def _base_subscriber_callback(self, msg, interface: str):
        to_print = 'interface: ' + interface + '\n'
        if self.csv:
            to_print += message_to_csv(msg, truncate_length=self.truncate_length,
                                       no_arr=self.no_arr, no_str=self.no_str)
        else:
            # The "easy" way to print out a representation here is to call message_to_yaml().
            # However, the message contains numbers for the event type, but we want to show
            # meaningful names to the user.  So we call message_to_ordereddict() instead,
            # and replace the numbers with meaningful names before dumping to YAML.
            msgdict = message_to_ordereddict(msg, truncate_length=self.truncate_length,
                                             no_arr=self.no_arr, no_str=self.no_str)

            if 'info' in msgdict:
                info = msgdict['info']
                if 'event_type' in info:
                    info['event_type'] = self._event_number_to_name[info['event_type']]

            to_print += yaml.dump(msgdict, allow_unicode=True, width=sys.maxsize,
                                  default_flow_style=self.flow_style)

            to_print += '---'
        try:
            self.output_msg_queue.put(to_print, timeout=0.5)
        except queue.Full:
            print('Output message is full! Please increase the queue size of output message by '
                  '"--queue_size"')
