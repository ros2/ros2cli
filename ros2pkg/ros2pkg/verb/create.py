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
import platform

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
            default=os.path.realpath(os.getcwd()),
            help='Directory where to create the package')
        parser.add_argument(
            '--build-tool',
            default='ament_cmake',
            choices=['cmake', 'ament_cmake'],
            help='Which build tool to use')
        parser.add_argument(
            '--dependencies',
            nargs='+',
            default=[],
            help='list of dependencies')
        parser.add_argument(
            '--maintainer-email',
            default=getpass.getuser() + '@' + platform.uname()[1] + '.local',
            help='email address of the maintainer of this package'),
        parser.add_argument(
            '--maintainer-name',
            default=getpass.getuser(),
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

        if args.cpp_node_name == args.cpp_library_name:
            raise ValueError('cpp_node_name has to be different from cpp_library_name')

        print('going to create a new package')
        print('package name:', args.package_name)
        print('destination directory', args.destination_directory)
        print('build tool:', args.build_tool)
        print('maintainer_email:', args.maintainer_email)
        print('maintainer_name:', args.maintainer_name)
        print('dependencies:', args.dependencies)
        print('cpp_node_name:', args.cpp_node_name)
        print('cpp_library_name:', args.cpp_library_name)

        package_directory = create_folder(args.package_name, args.destination_directory)
        if not package_directory:
            print('unable to create folder', args.destination_directory)
            return False

        package_xml_config = {
            'package_name': args.package_name,
            'maintainer_email': args.maintainer_email,
            'maintainer_name': args.maintainer_name,
            'dependencies': args.dependencies,
        }

        if args.build_tool == 'cmake':
            create_template_file(
                'cmake/package.xml.em',
                package_directory,
                'package.xml',
                package_xml_config)

            cmakelists_config = {
                'project_name': args.package_name,
                'dependencies': args.dependencies,
                'cpp_node_name': args.cpp_node_name,
                'cpp_library_name': args.cpp_library_name,
            }
            create_template_file(
                'cmake/CMakeLists.txt.em',
                package_directory,
                'CMakeLists.txt',
                cmakelists_config)

            cmake_config = {
                'project_name': args.package_name,
                'create_cpp_library': True if args.cpp_library_name else False,
                'create_cpp_exe': True if args.cpp_node_name else False,
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

        if args.build_tool == 'ament_cmake':
            create_template_file(
                'ament_cmake/package.xml.em',
                package_directory,
                'package.xml',
                package_xml_config)

            cmakelists_config = {
                'project_name': args.package_name,
                'dependencies': args.dependencies,
                'cpp_node_name': args.cpp_node_name + '.cpp',
                'cpp_library_name': args.cpp_library_name + '.cpp',
            }
            create_template_file(
                'ament_cmake/CMakeLists.txt.em',
                package_directory,
                'CMakeLists.txt',
                cmakelists_config)

        if args.cpp_node_name or args.cpp_library_name:
            src_folder = create_folder('src', package_directory)
            if not src_folder:
                print('unable to create folder', package_directory)
                return False

        if args.cpp_node_name:
            cpp_node_config = {
                'package_name': args.package_name,
            }
            create_template_file(
                'cpp/main.cpp.em',
                src_folder,
                args.cpp_node_name + '.cpp',
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

        return True
