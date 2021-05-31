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
import signal
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

    # on Windows Python scripts are invokable through the interpreter
    if os.name == 'nt' and path.endswith('.py'):
        cmd.insert(0, sys.executable)

    if prefix is not None:
        cmd = prefix + cmd

    process = subprocess.Popen(cmd)
    while process.returncode is None:
        try:
            process.communicate()
        except KeyboardInterrupt:
            # the subprocess will also receive the signal and should shut down
            # therefore we continue here until the process has finished
            pass
    if process.returncode != 0:
        if os.name == 'posix':
            # a negative value -N indicates that the child was terminated by signal N.
            print(signal.strsignal(-process.returncode))
        else:
            # print general failure message instead.
            print('Process exits failure')
    return process.returncode


class ExecutableNameCompleter:
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
