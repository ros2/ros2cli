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
import subprocess
import sys

from ros2pkg.api.create import create_folder
from ros2pkg.api.create import create_template_file
from ros2pkg.verb import VerbExtension


class CreateVerb(VerbExtension):
    """Create a new ROS2 package."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'package_name',
            help='The package name')
        parser.add_argument(
            '--destination-directory',
            default=os.curdir,
            help='Directory where to create the package directory')
        parser.add_argument(
            '--build-type',
            default='ament_cmake',
            choices=['cmake', 'ament_cmake'],
            help='The build type to process the package with')
        parser.add_argument(
            '--dependencies',
            nargs='+',
            default=[],
            help='list of dependencies')
        parser.add_argument(
            '--maintainer-email',
            nargs='?',
            help='email address of the maintainer of this package'),
        parser.add_argument(
            '--maintainer-name',
            nargs='?',
            help='name of the maintainer of this package'),
        parser.add_argument(
            '--cpp-node-name',
            nargs='?',
            help='name of the empty cpp executable')
        parser.add_argument(
            '--cpp-library-name',
            nargs='?',
            help='name of the empty cpp library')

    def main(self, *, args):

        maintainer_name = getpass.getuser()
        if not args.maintainer_name:
            # try getting the name from the global git config
            p = subprocess.Popen(
                ['git', 'config', '--global', 'user.name'],
                stdout=subprocess.PIPE)
            resp = p.communicate()
            name = resp[0].decode().rstrip()
            if name:
                maintainer_name = name
        else:
            maintainer_name = args.maintainer_name

        maintainer_email = maintainer_name + '@todo.todo'
        if not args.maintainer_email:
            # try getting the email from the global git config
            p = subprocess.Popen(
                ['git', 'config', '--global', 'user.email'],
                stdout=subprocess.PIPE)
            resp = p.communicate()
            email = resp[0].decode().rstrip()
            if email:
                maintainer_email = email
        else:
            maintainer_email = args.maintainer_email

        cpp_node_name = None
        if args.cpp_node_name:
            cpp_node_name = args.cpp_node_name
            if args.cpp_node_name == args.cpp_library_name:
                cpp_node_name = args.cpp_node_name + '_node'
                print('[WARNING] node name can not be equal to the library name', file=sys.stderr)
                print('[WARNING] renaming node to %s' % cpp_node_name, file=sys.stderr)

        print('going to create a new package')
        print('package name:', args.package_name)
        print('destination directory', args.destination_directory)
        print('build type:', args.build_type)
        print('maintainer_email:', maintainer_email)
        print('maintainer_name:', maintainer_name)
        if args.dependencies:
            print('dependencies:', args.dependencies)
        if args.cpp_node_name:
            print('cpp_node_name:', cpp_node_name)
        if args.cpp_library_name:
            print('cpp_library_name:', args.cpp_library_name)

        package_directory = create_folder(args.package_name, args.destination_directory)
        if not package_directory:
            return 'unable to create folder: ' + args.destination_directory

        package_xml_config = {
            'package_name': args.package_name,
            'maintainer_email': maintainer_email,
            'maintainer_name': maintainer_name,
            'dependencies': args.dependencies,
        }

        if args.build_type == 'cmake':
            create_template_file(
                'cmake/package.xml.em',
                package_directory,
                'package.xml',
                package_xml_config)

            cmakelists_config = {
                'project_name': args.package_name,
                'dependencies': args.dependencies,
                'cpp_node_name': cpp_node_name,
                'cpp_library_name': args.cpp_library_name,
            }
            create_template_file(
                'cmake/CMakeLists.txt.em',
                package_directory,
                'CMakeLists.txt',
                cmakelists_config)

            cmake_config = {
                'project_name': args.package_name,
                'cpp_library_name': args.cpp_library_name,
                'cpp_node_name': cpp_node_name,
            }
            create_template_file(
                'cmake/Config.cmake.in.em',
                package_directory,
                args.package_name + 'Config.cmake.in',
                cmake_config)

            version_config = {
                'project_name': args.package_name,
            }
            create_template_file(
                'cmake/ConfigVersion.cmake.in.em',
                package_directory,
                args.package_name + 'ConfigVersion.cmake.in',
                version_config)

        if args.build_type == 'ament_cmake':
            create_template_file(
                'ament_cmake/package.xml.em',
                package_directory,
                'package.xml',
                package_xml_config)

            cmakelists_config = {
                'project_name': args.package_name,
                'dependencies': args.dependencies,
                'cpp_node_name': cpp_node_name,
                'cpp_library_name': args.cpp_library_name,
            }
            create_template_file(
                'ament_cmake/CMakeLists.txt.em',
                package_directory,
                'CMakeLists.txt',
                cmakelists_config)

        if args.cpp_node_name or args.cpp_library_name:
            src_folder = create_folder('src', package_directory)
            if not src_folder:
                return 'unable to create folder: ' + args.destination_directory

        if args.cpp_node_name:
            cpp_node_config = {
                'package_name': args.package_name,
            }
            create_template_file(
                'cpp/main.cpp.em',
                src_folder,
                cpp_node_name + '.cpp',
                cpp_node_config)

        if args.cpp_library_name:
            include_folder = create_folder('include/' + args.package_name, package_directory)
            class_name = args.cpp_library_name.replace('_', ' ').title()
            class_name = ''.join(x for x in class_name if not x.isspace())
            cpp_header_config = {
                'package_name': args.package_name,
                'library_name': args.cpp_library_name,
                'class_name': class_name,
            }
            create_template_file(
                'cpp/header.hpp.em',
                include_folder,
                args.cpp_library_name + '.hpp',
                cpp_header_config)

            cpp_library_config = {
                'package_name': args.package_name,
                'library_name': args.cpp_library_name,
                'class_name': class_name
            }
            create_template_file(
                'cpp/library.cpp.em',
                src_folder,
                args.cpp_library_name + '.cpp',
                cpp_library_config)

            visibility_config = {
                'package_name': args.package_name.upper(),
            }
            create_template_file(
                'cpp/visibility_control.h.em',
                include_folder,
                'visibility_control.h',
                visibility_config)
