# Copyright 2022 Open Source Robotics Foundation, Inc.
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
import sys
from typing import TypeVar

import rclpy

from rclpy.qos import QoSPresetProfiles

from ros2cli.helpers import unsigned_int
from ros2cli.node.strategy import NodeStrategy
from ros2cli.qos import add_qos_arguments
from ros2cli.qos import profile_configure_short_keys

from ros2service.api import get_service_class
from ros2service.api import ServiceNameCompleter
from ros2service.api import ServiceTypeCompleter
from ros2service.verb import VerbExtension

from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_service
from service_msgs.msg import ServiceEventInfo

import yaml


DEFAULT_TRUNCATE_LENGTH = 128
MsgType = TypeVar('MsgType')


class EchoVerb(VerbExtension):
    """Echo a service."""

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
            'service_name',
            help="Name of the ROS service to echo (e.g. '/add_two_ints')")
        arg.completer = ServiceNameCompleter(
            include_hidden_services_key='include_hidden_services')
        arg = parser.add_argument(
            'service_type', nargs='?',
            help="Type of the ROS service (e.g. 'example_interfaces/srv/AddTwoInts')")
        arg.completer = ServiceTypeCompleter(service_name_key='service_name')
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
        add_qos_arguments(parser, 'service introspection', 'services_default')

    def main(self, *, args):
        if args.service_type is None:
            with NodeStrategy(args) as node:
                try:
                    srv_module = get_service_class(
                        node, args.service_name, include_hidden_services=True)
                except (AttributeError, ModuleNotFoundError, ValueError):
                    raise RuntimeError(f"The service name '{args.service_name}' is invalid")
        else:
            try:
                srv_module = get_service(args.service_type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError(f"The service type '{args.service_type}' is invalid")

        if srv_module is None:
            raise RuntimeError('Could not load the type for the passed service')

        event_msg_type = srv_module.Event

        # TODO(clalancette): We should probably expose this postfix from rclpy
        event_topic_name = args.service_name + '/_service_event'

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

        with NodeStrategy(args) as node:
            sub = node.create_subscription(
                event_msg_type,
                event_topic_name,
                self._subscriber_callback,
                qos_profile=qos_profile)

            have_printed_warning = False
            executor = rclpy.get_global_executor()
            try:
                executor.add_node(node)
                while executor.context.ok():
                    if not have_printed_warning and sub.get_publisher_count() < 1:
                        print(f"No publishers on topic '{event_topic_name}'; "
                              'is service introspection on the client or server enabled?')
                        have_printed_warning = True
                    executor.spin_once()
            finally:
                executor.remove_node(node)

            sub.destroy()

    def _subscriber_callback(self, msg):
        if self.csv:
            to_print = message_to_csv(msg, truncate_length=self.truncate_length,
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

            to_print = yaml.dump(msgdict, allow_unicode=True, width=sys.maxsize,
                                 default_flow_style=self.flow_style)

            to_print += '---'

        print(to_print)
