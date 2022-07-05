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
from typing import Optional
from typing import TypeVar

from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import unsigned_int
from rcl_interfaces.msg import ServiceEventType

from ros2service.api import ServiceNameCompleter, get_service_class
from ros2service.api import ServiceTypeCompleter
from ros2service.verb import VerbExtension

from rosidl_runtime_py import message_to_csv, message_to_ordereddict
from rosidl_runtime_py.utilities import get_message, get_service

import yaml

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
        self.client = None
        self.server = None
        self.truncate_length = None
        self.include_message_info = None
        self.srv_module = None
        self.event_enum = None
        self.event_msg_type = get_message("rcl_interfaces/msg/ServiceEvent")
        self.qos_profile = QoSPresetProfiles.get_from_short_key("services_default")
        self.__yaml_representer_registered = False

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'service_name',
            help="Name of the ROS service to echo from (e.g. '/add_two_ints')")
        arg.completer = ServiceNameCompleter(
            include_hidden_services_key='include_hidden_services')
        arg = parser.add_argument(
            'service_type', nargs='?',
            help="Type of the ROS service (e.g. 'std_srvs/srv/Empty')")
        arg.completer = ServiceTypeCompleter(
            service_name_key='service_name')
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
            '--csv', action='store_true',
            help=(
                'Output all recursive fields separated by commas (e.g. for '
                'plotting). '
                'If --include-message-info is also passed, the following fields are prepended: '
                'source_timestamp, received_timestamp, publication_sequence_number,'
                ' reception_sequence_number.'
            )
        )
        parser.add_argument(
            '--include-message-info', '-i', action='store_true',
            help='Shows the associated message info.')
        parser.add_argument(
            '--client', action='store_true', help="Echo only request sent or response received by service client")
        parser.add_argument(
            '--server', action='store_true', help="Echo only request received or response sent by service server")

    def main(self, *, args):
        self.truncate_length = args.truncate_length if not args.full_length else None
        self.no_arr = args.no_arr
        self.no_str = args.no_str
        self.csv = args.csv
        self.include_message_info = args.include_message_info
        self.client = args.client
        self.server = args.server

        if args.service_type is None:
            with NodeStrategy(args) as node:
                self.srv_module = get_service_class(
                    node, args.service_name, blocking=False, include_hidden_services=True)
        else:
            try:
                self.srv_module = get_service(args.service_type)
            except (AttributeError, ModuleNotFoundError, ValueError):
                raise RuntimeError("The service type '%s' is invalid" % args.service_type)

        if self.srv_module is None:
            raise RuntimeError(
                'Could not load the type for the passed service')

        event_topic_name = args.service_name + "/_service_event"

        with NodeStrategy(args) as node:
            self.subscribe_and_spin(
                node,
                event_topic_name,
                self.event_msg_type
            )

    def subscribe_and_spin(
            self,
            node: Node,
            event_topic_name: str,
            event_msg_type: MsgType,
    ) -> Optional[str]:
        """Initialize a node with a single subscription and spin."""

        node.create_subscription(
            event_msg_type,
            event_topic_name,
            self._subscriber_callback,
            self.qos_profile)

        rclpy.spin(node)

    def _subscriber_callback(self, msg, info):
        serialize_msg_type = None
        self.event_enum = msg.info.event_type

        if self.client and self.server:
            pass
        elif self.client:
            if self.event_enum is ServiceEventType.REQUEST_RECEIVED or \
                    self.event_enum is ServiceEventType.RESPONSE_SENT:
                return
        elif self.server:
            if self.event_enum is ServiceEventType.REQUEST_SENT or \
                    self.event_enum is ServiceEventType.RESPONSE_RECEIVED:
                return

        if self.event_enum is ServiceEventType.REQUEST_RECEIVED or \
                self.event_enum is ServiceEventType.REQUEST_SENT:
            serialize_msg_type = self.srv_module.Request
        elif self.event_enum is ServiceEventType.RESPONSE_RECEIVED or \
                self.event_enum is ServiceEventType.RESPONSE_SENT:
            serialize_msg_type = self.srv_module.Response
        else:  # TODO remove this once event enum is correct
            print("Returning invalid service event type")
            return

        # csv
        if self.csv:
            to_print = message_to_csv(
                msg,
                truncate_length=self.truncate_length,
                no_arr=self.no_arr,
                no_str=self.no_str,
                serialize_msg_type=serialize_msg_type
            )
            if self.include_message_info:
                to_print = f'{",".join(str(x) for x in info.values())},{to_print}'
            print(to_print)
            return

        # yaml
        if self.include_message_info:
            print(yaml.dump(info), end='---\n')
        print(
            self.format_output(message_to_ordereddict(
                msg, truncate_length=self.truncate_length,
                no_arr=self.no_arr, no_str=self.no_str, serialize_msg_type=serialize_msg_type)),
            end='---------------------------\n')

    def format_output(self, dict_service_event: OrderedDict):
        dict_output = OrderedDict()
        dict_output['stamp'] = dict_service_event['info']['stamp']
        dict_output['client_id'] = dict_service_event['info']['client_id']
        dict_output['sequence_number'] = dict_service_event['info']['sequence_number']

        if self.event_enum is ServiceEventType.REQUEST_SENT:
            dict_output['event_type'] = 'CLIENT_REQUEST_SENT'
        elif self.event_enum is ServiceEventType.RESPONSE_RECEIVED:
            dict_output['event_type'] = 'CLIENT_RESPONSE_RECEIVED'
        elif self.event_enum is ServiceEventType.REQUEST_RECEIVED:
            dict_output['event_type'] = 'SERVER_REQUEST_RECEIVED'
        elif self.event_enum is ServiceEventType.RESPONSE_SENT:
            dict_output['event_type'] = 'SERVER_RESPONSE_SENT'

        if self.event_enum is ServiceEventType.REQUEST_RECEIVED or \
                self.event_enum is ServiceEventType.REQUEST_SENT:
            dict_output['request'] = dict_service_event['serialized_event']
        elif self.event_enum is ServiceEventType.RESPONSE_RECEIVED or \
                self.event_enum is ServiceEventType.RESPONSE_SENT:
            dict_output['response'] = dict_service_event['serialized_event']

        # Register custom representer for YAML output
        if not self.__yaml_representer_registered:
            yaml.add_representer(OrderedDict, represent_ordereddict)
            self.__yaml_representer_registered = True

        return yaml.dump(dict_output,
                         allow_unicode=True,
                         width=sys.maxsize,
                         default_flow_style=self.flow_style,
                         )
