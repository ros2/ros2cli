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
import sys
from typing import TypeVar

from collections import OrderedDict
import sys
from typing import Optional, TypeVar

import rclpy
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.qos import QoSPresetProfiles
from rosidl_runtime_py import message_to_csv
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_service
from service_msgs.msg import ServiceEventInfo
import yaml

from ros2cli.node.strategy import NodeStrategy
from ros2service.api import get_service_class
from ros2service.api import ServiceNameCompleter
from ros2service.api import ServiceTypeCompleter
from ros2service.verb import VerbExtension
from ros2topic.api import unsigned_int

DEFAULT_TRUNCATE_LENGTH = 128
MsgType = TypeVar('MsgType')

def represent_ordereddict(dumper, data):
    items = []
    for k, v in data.items():
        items.append((dumper.represent_data(k), dumper.represent_data(v)))
    return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', items)

class EchoVerb(VerbExtension):
    """Echo a service."""

    def __init__(self):
        super().__init__()
        self.no_str = None
        self.no_arr = None
        self.csv = None
        self.flow_style = None
        self.client_only = None
        self.server_only = None
        self.truncate_length = None
        self.exclude_message_info = None
        self.srv_module = None
        self.event_enum = None
        self.qos_profile = QoSPresetProfiles.get_from_short_key("services_default")
        self.__yaml_representer_registered = False
        self.event_type_map = dict((v, k) for k, v in ServiceEventInfo._Metaclass_ServiceEventInfo__constants.items())

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
            '--full-length', '-f', action='store_true',
            help='Output all elements for arrays, bytes, and string with a '
                 "length > '--truncate-length', by default they are truncated "
                 "after '--truncate-length' elements with '...'")
        parser.add_argument(
            '--truncate-length', '-l', type=unsigned_int, default=DEFAULT_TRUNCATE_LENGTH,
            help='The length to truncate arrays, bytes, and string to '
                 f'(default: {DEFAULT_TRUNCATE_LENGTH})')
        parser.add_argument(
            '--no-arr', action='store_true', help="Don't print array fields of messages")
        parser.add_argument(
            '--no-str', action='store_true', help="Don't print string fields of messages")
        parser.add_argument(
            '--csv', action='store_true',
            help=(
                'Output all recursive fields separated by commas (e.g. for plotting).'
            ))
        parser.add_argument(
            '--exclude-message-info', action='store_true', help='Hide associated message info.')
        parser.add_argument(
            '--client-only', action='store_true', help="Echo only request sent or response received by service client")
        parser.add_argument(
            '--server-only', action='store_true', help="Echo only request received or response sent by service server")

    def main(self, *, args):
        self.truncate_length = args.truncate_length if not args.full_length else None
        self.no_arr = args.no_arr
        self.no_str = args.no_str
        self.csv = args.csv
        self.exclude_message_info = args.exclude_message_info
        self.client_only = args.client_only
        self.server_only = args.server_only
        event_topic_name = args.service_name + \
            _rclpy.service_introspection.RCL_SERVICE_INTROSPECTION_TOPIC_POSTFIX

        if self.server_only and self.client_only:
            raise RuntimeError("--client-only and --server-only are mutually exclusive")

        if args.service_type is None:
            with NodeStrategy(args) as node:
                try:
                    self.srv_module = get_service_class(
                        node, args.service_name, blocking=False, include_hidden_services=True)
                    self.event_msg_type = self.srv_module.Event
                except (AttributeError, ModuleNotFoundError, ValueError):
                    raise RuntimeError("The service name '%s' is invalid" % args.service_name)
        else:
            try:
                self.srv_module = get_service(args.service_type)
                self.event_msg_type = self.srv_module.Event
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError(f"The service type '{args.service_type}' is invalid")

        if self.srv_module is None:
            raise RuntimeError('Could not load the type for the passed service')

        with NodeStrategy(args) as node:
            self.subscribe_and_spin(
                node,
                event_topic_name,
                self.event_msg_type)

    def subscribe_and_spin(self, node, event_topic_name: str, event_msg_type: MsgType) -> Optional[str]:
        """Initialize a node with a single subscription and spin."""
        node.create_subscription(
            event_msg_type,
            event_topic_name,
            self._subscriber_callback,
            self.qos_profile)
        rclpy.spin(node)

    def _subscriber_callback(self, msg):
        if self.csv:
            print(self.format_csv_output(msg))
        else:
            print(self.format_yaml_output(msg))
        print('---------------------------')

    def format_csv_output(self, msg: MsgType):
        """Convert a message to a CSV string."""
        if self.exclude_message_info:
            msg.info = ServiceEventInfo()
        to_print = message_to_csv(
            msg,
            truncate_length=self.truncate_length,
            no_arr=self.no_arr,
            no_str=self.no_str)
        return to_print

    def format_yaml_output(self, msg: MsgType):
        """Pretty-format a service event message."""
        event_dict = message_to_ordereddict(
            msg,
            truncate_length=self.truncate_length,
            no_arr=self.no_arr,
            no_str=self.no_str)

        event_dict['info']['event_type'] = \
            self.event_type_map[event_dict['info']['event_type']]

        if self.exclude_message_info:
            del event_dict['info']

        # unpack Request, Response sequences
        if len(event_dict['request']) == 0:
            del event_dict['request']
        else:
            event_dict['request'] = event_dict['request'][0]

        if len(event_dict['response']) == 0:
            del event_dict['response']
        else:
            event_dict['response'] = event_dict['response'][0]

        # Register custom representer for YAML output
        if not self.__yaml_representer_registered:
            yaml.add_representer(OrderedDict, represent_ordereddict)
            self.__yaml_representer_registered = True

        return yaml.dump(event_dict,
                         allow_unicode=True,
                         width=sys.maxsize,
                         default_flow_style=self.flow_style)
