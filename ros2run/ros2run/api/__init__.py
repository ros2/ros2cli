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
import sys

from ros2pkg.api import get_executable_paths
from ros2pkg.api import PackageNotFound


class MultipleExecutables(Exception):

    def __init__(self, paths):
        self.paths = paths


def get_executable_path(*, package_name, executable_name):
    paths = get_executable_paths(package_name=package_name)
    paths2base = {}
    for p in paths:
        basename = os.path.basename(p)
        if basename == executable_name:
            # pick exact match
            paths2base[p] = basename
        elif sys.platform == 'win32':
            # check extensions listed in PATHEXT for match without extension
            pathext = os.environ.get('PATHEXT', '').lower().split(os.pathsep)
            ext = os.path.splitext(basename)[1].lower()
            if ext in pathext and basename[:-len(ext)] == executable_name:
                # pick match because of known extension
                paths2base[p] = basename
    if not paths2base:
        return None
    if len(paths2base) > 1:
        raise MultipleExecutables(paths2base.keys())
    return list(paths2base.keys())[0]


def run_executable(*, path, argv, prefix=None):
    cmd = [path] + argv
    if prefix is not None:
        cmd = prefix + cmd
    completed_process = subprocess.run(cmd)
    return completed_process.returncode


class ExecutableNameCompleter(object):
    """Callable returning a list of executable names within a package."""

    def __init__(self, *, package_name_key=None):
        self.package_name_key = package_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        package_name = getattr(parsed_args, self.package_name_key)
        try:
            paths = get_executable_paths(package_name=package_name)
        except PackageNotFound:
            return []
        return [os.path.basename(p) for p in paths]
