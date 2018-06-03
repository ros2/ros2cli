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

from ament_index_python import get_package_prefix
from ament_index_python import get_package_share_directory
from ament_index_python import get_packages_with_prefixes
from ament_index_python import PackageNotFoundError


class PackageNotFound(Exception):

    def __init__(self, package_name):
        self.package_name = package_name


def get_package_names():
    return get_packages_with_prefixes().keys()


def get_prefix_path(package_name):
    try:
        prefix_path = get_package_prefix(package_name)
    except PackageNotFoundError:
        return None
    return prefix_path


def get_share_directory(package_name):
    try:
        return get_package_share_directory(package_name)
    except PackageNotFoundError:
        return None


def get_executable_paths(*, package_name):
    prefix_path = get_prefix_path(package_name)
    if prefix_path is None:
        raise PackageNotFound(package_name)
    base_path = os.path.join(prefix_path, 'lib', package_name)
    executable_paths = []
    for dirpath, dirnames, filenames in os.walk(base_path):
        # ignore folder starting with .
        dirnames[:] = [d for d in dirnames if d[0] not in ['.']]
        dirnames.sort()
        # select executable files
        for filename in sorted(filenames):
            path = os.path.join(dirpath, filename)
            if os.access(path, os.X_OK):
                executable_paths.append(path)
    return executable_paths


def package_name_completer(**kwargs):
    """Callable returning a list of packages names."""
    return get_package_names()
