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

from ament_index_python import get_resource
from ament_index_python import get_resources
from ament_index_python import has_resource
from ros2msg.api import get_all_message_types
from ros2msg.api import get_message_types
from ros2srv.api import get_all_service_types
from ros2srv.api import get_service_types


def get_all_types():
    all_types = []
    for package_name in get_resources('rosidl_interfaces'):
        service_types = get_service_types(package_name)
        if service_types:
            all_types.append(package_name)
    for package_name in get_resources('rosidl_interfaces'):
        message_types = get_message_types(package_name)
        if message_types:
            all_types.append(package_name)
    return all_types


def get_types(package_name):
    if not has_resource('packages', package_name):
        raise LookupError('Unknown Packs')
    try:
        content, _ = get_resource('rosidl_interfaces', package_name)
    except LookupError:
        return []
    interface_names = content.splitlines()
    msgsList = list(sorted({
        n[0:-4]
        for n in interface_names
        if n.startswith('msg/') and n[-4:] in ('.idl', '.msg')}))
    srvsList = list(sorted({
        n[0:-4]
        for n in interface_names
        if n.startswith('srv/') and n[-4:] in ('.idl', '.srv')}))
    typeList = []
    for item in msgsList:
        typeList.append(item)
    for item in srvsList:
        typeList.append(item)
    return typeList


def package_name_completer(**kwargs):
    """Callable returning a list of package name containing messages, services, and action."""
    return get_all_types()


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


# class NameCompleter:
#     """Callable returning a list of msgs, srvs, and/or action names within a package."""

#     def __init__(self, *, package_name_key=None):
#         self.package_name_key = package_name_key

#     def __call__(self, prefix, parsed_agrs, **kwargs):
#         pacakge_name = getattr(parsed_args, self.package_namep_key)
#         #this should only be returning the srvs for now
#         return get_types(package_name)
