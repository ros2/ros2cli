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
import importlib
from typing import Optional
from typing import TypeVar

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile
from rclpy.serialization import deserialize_message

from rcl_interfaces.msg import ServiceEventType
from rcl_interfaces.msg import ServiceEvent

from ros2topic.api import unsigned_int

from ros2service.api import ServiceNameCompleter
from ros2service.api import ServiceTypeCompleter
from ros2service.verb import VerbExtension
from rosidl_runtime_py.utilities import get_message
from ros2cli.node.strategy import NodeStrategy

from rosidl_runtime_py import message_to_yaml, message_to_csv
import yaml

DEFAULT_TRUNCATE_LENGTH = 128
MsgType = TypeVar('MsgType')


class EchoVerb(VerbExtension):
    """Echo a service."""

    def __init__(self):
        super().__init__()
        self.srv_module = None
        self.topic_name = None
        self.no_str = None
        self.no_arr = None
        self.truncate_length = None
        self.flow_style = None
        self.csv = None
        self.include_message_info = None
        self.hidden_topic_suffix = "/_service_event"
        self.message_type = get_message("rcl_interfaces/msg/ServiceEvent")
        self.qos_profile = QoSPresetProfiles.get_from_short_key("services_default")

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

    def main(self, *, args):
        self.topic_name = args.service_name + self.hidden_topic_suffix
        self.truncate_length = args.truncate_length if not args.full_length else None
        self.no_arr = args.no_arr
        self.no_str = args.no_str
        self.csv = args.csv
        self.include_message_info = args.include_message_info

        try:
            parts = args.service_type.split('/')
            if len(parts) == 2:
                parts = [parts[0], 'srv', parts[1]]
            package_name = parts[0]
            module = importlib.import_module('.'.join(parts[:-1]))
            srv_name = parts[-1]
            self.srv_module = getattr(module, srv_name)
        except (AttributeError, ModuleNotFoundError, ValueError):
            raise RuntimeError('The passed service type is invalid')
        try:
            var = self.srv_module.Request
            var = self.srv_module.Response
        except AttributeError:
            raise RuntimeError('The passed type is not a service')

        with NodeStrategy(args) as node:
            self.subscribe_and_spin(
                node,
                self.topic_name,
                self.message_type
            )

    def subscribe_and_spin(
            self,
            node: Node,
            topic_name: str,
            message_type: MsgType,
    ) -> Optional[str]:
        """Initialize a node with a single subscription and spin."""

        node.create_subscription(
            message_type,
            topic_name,
            self._subscriber_callback,
            self.qos_profile)

        rclpy.spin(node)

    def _subscriber_callback(self, msg, info):
        service_event_type = msg.info.event_type
        serialize_msg_type = None

        if service_event_type is ServiceEventType.REQUEST_RECEIVED or \
                service_event_type is ServiceEventType.REQUEST_SENT:
            serialize_msg_type = self.srv_module.Request
        elif service_event_type is ServiceEventType.RESPONSE_RECEIVED or \
                service_event_type is ServiceEventType.RESPONSE_SENT:
            serialize_msg_type = self.srv_module.Response
        else:  # TODO remove this else condition later
            print("# Invalid Type")
            return

        # csv: TODO
        if self.csv:
            to_print = message_to_csv(
                msg,
                truncate_length=self.truncate_length,
                no_arr=self.no_arr,
                no_str=self.no_str)
            if self.include_message_info:
                to_print = f'{",".join(str(x) for x in info.values())},{to_print}'
            print(to_print)
            return

        # yaml
        if self.include_message_info:
            print(yaml.dump(info), end='---\n')
        print(
            message_to_yaml(
                msg, truncate_length=self.truncate_length,
                no_arr=self.no_arr, no_str=self.no_str, flow_style=self.flow_style,
                serialize_msg_type=serialize_msg_type),
            end='---\n')
