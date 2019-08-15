# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from ros2cli.node.strategy import NodeStrategy
from ros2srv.api import service_type_completer
from rosidl_runtime_py.convert import message_to_yaml
from rosidl_runtime_py.utilities import get_service


def get_service_names_and_types(*, node, include_hidden_services=False):
    service_names_and_types = node.get_service_names_and_types()
    if not include_hidden_services:
        service_names_and_types = [
            (n, t) for (n, t) in service_names_and_types
            if not topic_or_service_is_hidden(n)]
    return service_names_and_types


def get_service_names(*, node, include_hidden_services=False):
    service_names_and_types = get_service_names_and_types(
        node=node, include_hidden_services=include_hidden_services)
    return [n for (n, t) in service_names_and_types]


class ServiceNameCompleter:
    """Callable returning a list of service names."""

    def __init__(self, *, include_hidden_services_key=None):
        self.include_hidden_services_key = include_hidden_services_key

    def __call__(self, prefix, parsed_args, **kwargs):
        with NodeStrategy(parsed_args) as node:
            return get_service_names(
                node=node,
                include_hidden_services=getattr(
                    parsed_args, self.include_hidden_services_key))


class ServiceTypeCompleter:
    """Callable returning an existing service type or all service types."""

    def __init__(self, *, service_name_key=None):
        self.service_name_key = service_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        if self.service_name_key is not None:
            with NodeStrategy(parsed_args) as node:
                service_name = getattr(parsed_args, self.service_name_key)
                names_and_types = get_service_names_and_types(
                    node=node, include_hidden_services=True)
                for n, t in names_and_types:
                    if n == service_name:
                        return t
        return service_type_completer()


class ServicePrototypeCompleter:
    """Callable returning a service prototype."""

    def __init__(self, *, service_type_key=None):
        self.service_type_key = service_type_key

    def __call__(self, prefix, parsed_args, **kwargs):
        service = get_service(getattr(parsed_args, self.service_type_key))
        return [message_to_yaml(service.Request())]
