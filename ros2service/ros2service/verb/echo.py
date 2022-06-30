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

from ros2topic.api import unsigned_int

from ros2service.api import ServiceNameCompleter
from ros2service.api import ServiceTypeCompleter
from ros2service.verb import VerbExtension
from rosidl_runtime_py.utilities import get_message
from ros2cli.node.strategy import NodeStrategy

from rosidl_runtime_py import message_to_yaml
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
            '--include-message-info', '-i', action='store_true',
            help='Shows the associated message info.')

    def main(self, *, args):
        self.topic_name = args.service_name + self.hidden_topic_suffix
        self.truncate_length = args.truncate_length if not args.full_length else None
        self.no_arr = args.no_arr
        self.no_str = args.no_str

        try:
            parts = args.service_type.split('/')
            if len(parts) == 2:
                parts = [parts[0], 'srv', parts[1]]
            package_name = parts[0]
            print(parts[:-1])
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
        serialized_event = b''.join(msg.serialized_event)

        if service_event_type is ServiceEventType.REQUEST_RECEIVED or \
                service_event_type is ServiceEventType.REQUEST_SENT:
            service_request = deserialize_message(serialized_event,
                                                  self.srv_module.Request)
        elif service_event_type is ServiceEventType.RESPONSE_RECEIVED or \
                service_event_type is ServiceEventType.RESPONSE_SENT:
            service_request = deserialize_message(serialized_event,
                                                  self.srv_module.Response)
