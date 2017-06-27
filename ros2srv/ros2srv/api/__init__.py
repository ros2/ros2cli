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

import os

from ament_index_python import get_resource
from ament_index_python import get_resources
from ament_index_python import has_resource


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
    return [n[:-4] for n in interface_names if n.endswith('.srv')]


def get_service_path(package_name, service_name):
    service_types = get_service_types(package_name)
    if service_name not in service_types:
        raise LookupError('Unknown service name')
    prefix_path = has_resource('packages', package_name)
    # TODO(dirk-thomas) this logic should come from a rosidl related package
    return os.path.join(
        prefix_path, 'share', package_name, 'srv', service_name + '.srv')


def service_package_name_completer(**kwargs):
    """Callable returning a list of package names which contain services."""
    return get_all_service_types().keys()


def service_type_completer(**kwargs):
    """Callable returning a list of service types."""
    service_types = []
    for package_name, service_names in get_all_service_types().items():
        for service_name in service_names:
            service_types.append(
                '{package_name}/{service_name}'.format_map(locals()))
    return service_types


class ServiceNameCompleter(object):
    """Callable returning a list of service names within a package."""

    def __init__(self, *, package_name_key=None):
        self.package_name_key = package_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        package_name = getattr(parsed_args, self.package_name_key)
        return get_service_types(package_name)
