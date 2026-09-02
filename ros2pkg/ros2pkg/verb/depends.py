# Copyright 2026 Miko Parkkinen.
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

from ament_index_python import get_package_share_directory
from ament_index_python import PackageNotFoundError
from catkin_pkg.package import parse_package

from ros2pkg.api import package_name_completer
from ros2pkg.verb import VerbExtension


PACKAGE_NOT_FOUND = 'Package not found'
PACKAGE_XML_NOT_FOUND = 'Package XML manifest not found'

DEPENDENCY_ATTRIBUTES = (
    'build_depends',
    'buildtool_depends',
    'build_export_depends',
    'buildtool_export_depends',
    'exec_depends',
    'test_depends',
    'doc_depends',
)


class DependsVerb(VerbExtension):
    """Output the direct dependencies declared by a package."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name',
            help='The package name')
        arg.completer = package_name_completer

    def main(self, *, args):
        try:
            package_share_dir = get_package_share_directory(args.package_name)
        except PackageNotFoundError:
            return PACKAGE_NOT_FOUND

        package_xml = os.path.join(package_share_dir, 'package.xml')
        if not os.path.isfile(package_xml):
            return PACKAGE_XML_NOT_FOUND

        package = parse_package(package_xml)
        package.evaluate_conditions(os.environ)

        dependencies = {
            dependency.name
            for attribute in DEPENDENCY_ATTRIBUTES
            for dependency in getattr(package, attribute)
            if dependency.evaluated_condition
        }
        for dependency in sorted(dependencies):
            print(dependency)
