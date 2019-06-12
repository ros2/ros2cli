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

import xml.etree.ElementTree as ET

from ament_index_python import get_package_share_directory
from ament_index_python import PackageNotFoundError
from ros2pkg.api import package_name_completer
from ros2pkg.verb import VerbExtension


PACKAGE_NOT_FOUND = 'Package not found'
PACKAGE_XML_NOT_FOUND = 'Package XML manifest not found'
PACKAGE_XML_TAG_NOT_FOUND = 'XML tag not found'


class XmlVerb(VerbExtension):
    """Output the XML of the package manifest or a specific tag."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name',
            help='The package name')
        arg.completer = package_name_completer
        parser.add_argument(
            '-t', '--tag',
            help="The XML tag to output (e.g. 'version')")

    def main(self, *, args):
        try:
            package_share_dir = get_package_share_directory(args.package_name)
        except PackageNotFoundError:
            return PACKAGE_NOT_FOUND

        package_xml = os.path.join(package_share_dir, 'package.xml')
        if not os.path.isfile(package_xml):
            return PACKAGE_XML_NOT_FOUND

        tree = ET.parse(package_xml)
        if args.tag is None:
            ET.dump(tree)
            return 0

        elements = tree.getroot().findall(args.tag)
        if not elements:
            return PACKAGE_XML_TAG_NOT_FOUND

        for element in elements:
            print(element.text)
