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
import shutil
import subprocess
import sys

from catkin_pkg.package import Dependency
from catkin_pkg.package import Export
from catkin_pkg.package import Package
from catkin_pkg.package import Person

from ros2pkg.api.create import create_package_environment
from ros2pkg.api.create import populate_ament_cmake
from ros2pkg.api.create import populate_ament_python
from ros2pkg.api.create import populate_cmake
from ros2pkg.api.create import populate_cpp_library
from ros2pkg.api.create import populate_cpp_node
from ros2pkg.api.create import populate_python_libary
from ros2pkg.api.create import populate_python_node

from ros2pkg.verb import VerbExtension


class CreateVerb(VerbExtension):
    """Create a new ROS 2 package."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'package_name',
            help='The package name')
        parser.add_argument(
            '--package-format',
            '--package_format',
            type=int,
            default=3,
            choices=[2, 3],
            help='The package.xml format.')
        parser.add_argument(
            '--description',
            default='TODO: Package description',
            help='The description given in the package.xml')
        parser.add_argument(
            '--license',
            default='TODO: License declaration',
            help='The license attached to this package')
        parser.add_argument(
            '--destination-directory',
            default=os.curdir,
            help='Directory where to create the package directory')
        parser.add_argument(
            '--build-type',
            default='ament_cmake',
            choices=['cmake', 'ament_cmake', 'ament_python'],
            help='The build type to process the package with')
        parser.add_argument(
            '--dependencies',
            nargs='+',
            default=[],
            help='list of dependencies')
        parser.add_argument(
            '--maintainer-email',
            help='email address of the maintainer of this package'),
        parser.add_argument(
            '--maintainer-name', default=getpass.getuser(),
            help='name of the maintainer of this package'),
        parser.add_argument(
            '--node-name',
            help='name of the empty executable')
        parser.add_argument(
            '--library-name',
            help='name of the empty library')

    def main(self, *, args):
        maintainer = Person(args.maintainer_name)

        if args.maintainer_email:
            maintainer.email = args.maintainer_email
        else:
            # try getting the email from the global git config
            git = shutil.which('git')
            if git is not None:
                p = subprocess.Popen(
                    [git, 'config', '--global', 'user.email'],
                    stdout=subprocess.PIPE)
                resp = p.communicate()
                email = resp[0].decode().rstrip()
                if email:
                    maintainer.email = email
            if not maintainer.email:
                maintainer.email = maintainer.name + '@todo.todo'

        node_name = None
        library_name = None
        if args.library_name:
            library_name = args.library_name
        if args.node_name:
            node_name = args.node_name
            if args.node_name == args.library_name:
                node_name = args.node_name + '_node'
                print('[WARNING] node name can not be equal to the library name', file=sys.stderr)
                print('[WARNING] renaming node to %s' % node_name, file=sys.stderr)

        buildtool_depends = []
        if args.build_type == 'ament_cmake':
            if args.library_name:
                buildtool_depends = ['ament_cmake_ros']
            else:
                buildtool_depends = ['ament_cmake']

        test_dependencies = []
        if args.build_type == 'ament_cmake':
            test_dependencies = ['ament_lint_auto', 'ament_lint_common']
        if args.build_type == 'ament_python':
            test_dependencies = ['ament_copyright', 'ament_flake8', 'ament_pep257',
                                 'python3-pytest']

        if args.build_type == 'ament_python' and args.package_name == 'test':
            # If the package name is 'test', there will be a conflict between
            # the directory the source code for the package goes in and the
            # directory the tests for the package go in.
            return "Aborted since 'ament_python' packages can't be named 'test'. Please " + \
                'choose a different package name.'

        package = Package(
            package_format=args.package_format,
            name=args.package_name,
            version='0.0.0',
            description=args.description,
            maintainers=[maintainer],
            licenses=[args.license],
            buildtool_depends=[Dependency(dep) for dep in buildtool_depends],
            build_depends=[Dependency(dep) for dep in args.dependencies],
            test_depends=[Dependency(dep) for dep in test_dependencies],
            exports=[Export('build_type', content=args.build_type)]
        )

        package_path = os.path.join(args.destination_directory, package.name)
        if os.path.exists(package_path):
            return '\nAborted!\nThe directory already exists: ' + package_path + '\nEither ' + \
                'remove the directory or choose a different destination directory or package name'

        print('going to create a new package')
        print('package name:', package.name)
        print('destination directory:', os.path.abspath(args.destination_directory))
        print('package format:', package.package_format)
        print('version:', package.version)
        print('description:', package.description)
        print('maintainer:', [str(maintainer) for maintainer in package.maintainers])
        print('licenses:', package.licenses)
        print('build type:', package.get_build_type())
        print('dependencies:', [str(dependency) for dependency in package.build_depends])
        if node_name:
            print('node_name:', node_name)
        if library_name:
            print('library_name:', library_name)

        package_directory, source_directory, include_directory = \
            create_package_environment(package, args.destination_directory)
        if not package_directory:
            return 'unable to create folder: ' + args.destination_directory

        if args.build_type == 'cmake':
            populate_cmake(package, package_directory, node_name, library_name)

        if args.build_type == 'ament_cmake':
            populate_ament_cmake(package, package_directory, node_name, library_name)

        if args.build_type == 'ament_python':
            if not source_directory:
                return 'unable to create source folder in ' + args.destination_directory
            populate_ament_python(package, package_directory, source_directory, node_name)
            if node_name:
                populate_python_node(package, source_directory, node_name)
            if library_name:
                populate_python_libary(package, source_directory, library_name)

        if args.build_type == 'ament_cmake' or args.build_type == 'cmake':
            if node_name:
                if not source_directory:
                    return 'unable to create source folder in ' + args.destination_directory
                populate_cpp_node(package, source_directory, node_name)
            if library_name:
                if not source_directory or not include_directory:
                    return 'unable to create source or include folder in ' + \
                            args.destination_directory
                populate_cpp_library(
                    package,
                    source_directory,
                    include_directory,
                    library_name
                )
