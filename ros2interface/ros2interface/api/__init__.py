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

from rosidl_runtime_py import get_action_interfaces
from rosidl_runtime_py import get_interface_packages
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import get_service_interfaces
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py import utilities


def package_name_completer(**kwargs):
    """Callable returning a list of packages containing messages, services, and action."""
    return get_interface_packages()


def type_completer(**kwargs):
    """Callable returning a list of message, service, and action types."""
    types = []
    for package_name, service_names in get_service_interfaces().items():
        for service_name in service_names:
            types.append(f'{package_name}/{service_name}')

    for package_name, message_names in get_message_interfaces().items():
        for message_name in message_names:
            types.append(f'{package_name}/{message_name}')

    for package_name, action_names in get_action_interfaces().items():
        for action_name in action_names:
            types.append(f'{package_name}/{action_name}')
    return sorted(types)


def interface_to_yaml(identifier):
    interface = utilities.get_interface(identifier)
    if utilities.is_action(interface):
        instance = interface.Goal()
    elif utilities.is_service(interface):
        instance = interface.Request()
    else:
        instance = interface()

    return message_to_yaml(instance)
