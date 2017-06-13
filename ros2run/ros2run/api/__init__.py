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
import subprocess

from ros2cli.node.strategy import NodeStrategy
from ros2pkg.api import get_prefix_path


class MultipleExecutables(Exception):

    def __init__(self, paths):
        self.paths = paths


class PackageNotFound(Exception):

    def __init__(self, package_name):
        self.package_name = package_name


def get_executable_paths(*, node, package_name):
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


def get_executable_path(*, node, package_name, executable_name):
    paths = get_executable_paths(node=node, package_name=package_name)
    paths2base = {
        p: os.path.basename(p) for p in paths
        if os.path.basename(p) == executable_name}
    if not paths2base:
        return None
    if len(paths2base) > 1:
        raise MultipleExecutables(paths2base.keys())
    return list(paths2base.keys())[0]


def run_executable(*, path, argv):
    completed_process = subprocess.run([path] + argv)
    return completed_process.returncode


class ExecutableNameCompleter(object):
    """Callable returning a list of executable names within a package."""

    def __init__(self, *, package_name_key=None):
        self.package_name_key = package_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        package_name = getattr(parsed_args, self.package_name_key)
        with NodeStrategy(parsed_args) as node:
            try:
                paths = get_executable_paths(
                    node=node, package_name=package_name)
            except PackageNotFound:
                return []
            return [os.path.basename(p) for p in paths]
