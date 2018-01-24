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

from ros2cli.command import CommandExtension
from ros2pkg.api import get_prefix_path
from ros2pkg.api import package_name_completer
from ros2pkg.api import PackageNotFound
from ament_tools.packages import find_packages

def get_package_src_path(basepath, package_name):
    """
    Return the package source path
    :param basepath:
    :param pkg_name:
    :return: path:
    """
    path = basepath
    pkg_paths = find_packages(basepath)
    #print("package basepath " + basepath)
    for pkg_path, pkg in pkg_paths.items():
        #print("packages "+pkg_path+" "+pkg.name)
        if pkg.name == package_name:
            path = os.path.join(basepath, pkg_path)
            break

    return path

class CdCommand(CommandExtension):
    """Cd to a package src folder."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name',
            help='Name of the ROS package')
        arg.completer = package_name_completer

    def main(self, *, parser, args):
        path = get_prefix_path(
            package_name=args.package_name)
        if path is None:
            raise PackageNotFound(args.package_name)

        #print("package path : " + path)
        [head,tail] = os.path.split(path)
        if tail == "install":
            path = get_package_src_path(os.path.join(head, "src"), args.package_name)
        else:
            path = get_package_src_path(path, args.package_name)

        exit(path)