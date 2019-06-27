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
import os

from ament_index_python import get_resource
from ament_index_python import get_resources
from ament_index_python import has_resource


def get_all_interface_packages():
    return get_resources('rosidl_interfaces')


def get_interface(package_name):
    if not has_resource('packages', package_name):
        raise LookupError('Unknown package {}'.format(package_name))
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    type_list = set()
    for n in interface_names:
        split = n.rsplit('.', 1)
        if '_' not in split[0]:
            type_list.add(split[0])
    return sorted(type_list)


def get_interface_path(parts):
    prefix_path = has_resource('packages', parts[0])
    joined = '/'.join(parts)
    if len(parts[-1].rsplit('.', 1)) == 1:
        joined += '.idl'
    return os.path.join(
        prefix_path, 'share', joined)


def package_name_completer(**kwargs):
    """Callable returning a list of types containing messages, services, and action."""
    return get_all_interface_packages()


def type_completer(**kwargs):
    """Callable returning a list of message, service, and action types."""
    types = []
    for package_name, service_names in get_all_service_types().items():
        for service_name in service_names:
            types.append(
                '{package_name}/srv/{service_name}'.format_map(locals()))

    for package_name, message_names in get_all_message_types().items():
        for message_name in message_names:
            types.append(
                '{package_name}/msg/{message_name}'.format_map(locals()))
    return sorted(types)


def get_all_action_types():
    all_action_types = {}
    for package_name in get_resources('rosidl_interfaces'):
        action_types = get_action_types(package_name)
        if action_types:
            all_action_types[package_name] = action_types
    return all_action_types


def get_action_types(package_name):
    if not has_resource('packages', package_name):
        raise LookupError('Unknown package name')
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    # TODO(jacobperron) this logic should come from a rosidl related package
    # Only return actions in action folder
    return list(sorted({
        n[7:-7]
        for n in interface_names
        if n.startswith('action/') and n[-7:] in ('.action')}))


def get_all_message_types():
    all_message_types = {}
    for package_name in get_resources('rosidl_interfaces'):
        message_types = get_message_types(package_name)
        if message_types:
            all_message_types[package_name] = message_types
    return all_message_types


def get_message_types(package_name):
    if not has_resource('packages', package_name):
        raise LookupError('Unknown package name')
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    # TODO(dirk-thomas) this logic should come from a rosidl related package
    # Only return messages in msg folder
    return list(sorted({
        n[4:-4]
        for n in interface_names
        if n.startswith('msg/') and n[-4:] in ('.idl', '.msg')}))


def get_all_service_types():
    all_service_types = {}
    for package_name in get_resources('rosidl_interfaces'):
        service_types = get_service_types(package_name)
        if service_types:
            all_service_types[package_name] = service_types
    return all_service_types


def get_service_types(package_name):
    if not has_resource('packages', package_name):
        raise LookupError('Unknown package name')
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    # TODO(dirk-thomas) this logic should come from a rosidl related package
    # Only return services in srv folder
    return list(sorted({
        n[4:-4]
        for n in interface_names
        if n.startswith('srv/') and n[-4:] in ('.idl', '.srv')}))


def get_message_path(package_name, message_name):
    message_types = get_message_types(package_name)
    if message_name not in message_types:
        raise LookupError('Unknown message name')
    prefix_path = has_resource('packages', package_name)
    # TODO(dirk-thomas) this logic should come from a rosidl related package
    return os.path.join(
        prefix_path, 'share', package_name, 'msg', message_name + '.msg')
