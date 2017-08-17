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

import getpass
import os

from ros2pkg.api.create_api import create_folder
from ros2pkg.api.create_api import create_template_file
from ros2pkg.verb import VerbExtension

class CreateVerb(VerbExtension):
    """Create a new ROS2 package."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'package_name',
            help='The package name')
        arg = parser.add_argument(
            '--destination-directory',
            default=os.path.realpath(os.getcwd()),
            help='Directory where to create the package')
        arg = parser.add_argument(
            '--build-tool',
            default='ament_cmake',
            choices=['cmake', 'ament_cmake'],
            help='Which build tool to use')
        arg = parser.add_argument(
            '--dependencies',
            nargs='+',
            default=[],
            help='list of dependencies')
        arg = parser.add_argument(
            '--maintainer-email',
            default=getpass.getuser()+'@'+os.uname()[1]+'.local',
            help='email address of the maintainer of this package'),
        arg = parser.add_argument(
            '--maintainer-name',
            default=getpass.getuser(),
            help='name of the maintainer of this package'),
        arg = parser.add_argument(
            '--create-cpp-exe',
            nargs='?',
            const=True,
            required=False,
            help='create an empty cpp executable')
        arg = parser.add_argument(
            '--cpp-exe-name',
            nargs='+',
            default='main.cpp',
            help='name of the empty cpp executable')

    def main(self, *, args):
        package_name = args.package_name
        destination_directory = args.destination_directory
        build_tool = args.build_tool
        dependencies = args.dependencies
        maintainer_email = args.maintainer_email
        maintainer_name = args.maintainer_name
        create_cpp_exe = True if args.create_cpp_exe else False
        cpp_exe_name = args.cpp_exe_name

        print('going to create a new package')
        print('package name:', package_name)
        print('destination directory', destination_directory)
        print('build tool:', build_tool)
        print('maintainer_email:', maintainer_email)
        print('maintainer_name:', maintainer_name)
        print('dependencies:', dependencies)
        print('create_cpp_exe:', create_cpp_exe)
        print('cpp_exe_name:', cpp_exe_name)

        package_directory = create_folder(package_name, destination_directory)
        if not package_directory:
            return

        package_xml_config = {
            'package_name' : package_name,
            'build_tool' : build_tool,
            'maintainer_email' : maintainer_email,
            'maintainer_name' : maintainer_name,
            'dependencies' : dependencies,
            }
        create_template_file('package.xml.em', package_directory, 'package.xml', package_xml_config)

        cmakelists_config = {
            'project_name' : package_name,
            'dependencies' : dependencies,
            'create_cpp_exe' : create_cpp_exe,
            'cpp_exe_name' : cpp_exe_name
            }
        create_template_file('CMakelists.txt.em', package_directory, 'CMakelists.txt', cmakelists_config)

        if (args.create_cpp_exe):
            src_folder = create_folder('src', package_directory)
            if not src_folder:
                return

            cpp_exe_config = {
                'package_name' : package_name,
                }
            create_template_file('main.cpp.em', src_folder, cpp_exe_name, cpp_exe_config)
