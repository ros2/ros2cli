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

import contextlib
import os
import tempfile
import unittest
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.tools

import pytest


some_cli_packages = [
    'ros2cli',
    'ros2pkg'
]


@pytest.mark.rostest
@launch_testing.markers.keep_alive
def generate_test_description():
    return LaunchDescription([launch_testing.actions.ReadyToTest()])


class TestROS2PkgCLI(unittest.TestCase):

    @classmethod
    def setUpClass(
        cls,
        launch_service,
        proc_info,
        proc_output
    ):
        @contextlib.contextmanager
        def launch_pkg_command(self, arguments, **kwargs):
            pkg_command_action = ExecuteProcess(
                cmd=['ros2', 'pkg', *arguments],
                additional_env={'PYTHONUNBUFFERED': '1'},
                name='ros2pkg-cli',
                output='screen',
                **kwargs
            )
            with launch_testing.tools.launch_process(
                launch_service, pkg_command_action, proc_info, proc_output
            ) as pkg_command:
                yield pkg_command
        cls.launch_pkg_command = launch_pkg_command

    def test_list_packages(self):
        with self.launch_pkg_command(arguments=['list']) as pkg_command:
            assert pkg_command.wait_for_shutdown(timeout=2)
        assert pkg_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = pkg_command.output.splitlines()
        assert all(pkg in output_lines for pkg in some_cli_packages)

    def test_package_prefix(self):
        with self.launch_pkg_command(arguments=['prefix', 'ros2cli']) as pkg_command:
            assert pkg_command.wait_for_shutdown(timeout=2)
        assert pkg_command.exit_code == launch_testing.asserts.EXIT_OK
        output_lines = pkg_command.output.splitlines()
        assert len(output_lines) == 1
        prefix_path = output_lines[0]
        assert os.path.isdir(prefix_path)

    def test_not_a_package_prefix(self):
        with self.launch_pkg_command(arguments=['prefix', 'not_a_package']) as pkg_command:
            assert pkg_command.wait_for_shutdown(timeout=2)
        assert pkg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Package not found'],
            text=pkg_command.output,
            strict=True
        )

    def test_xml(self):
        with self.launch_pkg_command(arguments=['xml', 'ros2cli']) as pkg_command:
            assert pkg_command.wait_for_shutdown(timeout=2)
        assert pkg_command.exit_code == launch_testing.asserts.EXIT_OK
        root = ET.XML(pkg_command.output)
        assert root.tag == 'package'
        assert root.find('name').text == 'ros2cli'

    def test_not_a_package_xml(self):
        with self.launch_pkg_command(arguments=['xml', 'not_a_package']) as pkg_command:
            assert pkg_command.wait_for_shutdown(timeout=2)
        assert pkg_command.exit_code == 1
        assert launch_testing.tools.expect_output(
            expected_lines=['Package not found'],
            text=pkg_command.output,
            strict=True
        )

    def test_create_package(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            with self.launch_pkg_command(
                arguments=[
                    'create', 'a_test_package',
                    '--package-format', '3',
                    '--description', 'A test package dummy description',
                    '--license', 'Apache License 2.0',
                    '--build-type', 'ament_cmake',
                    '--dependencies', 'ros2pkg',
                    '--maintainer-email', 'nobody@nowhere.com',
                    '--maintainer-name', 'Nobody',
                    '--node-name', 'test_node',
                    '--library-name', 'test_library'
                ], cwd=tmpdir
            ) as pkg_command:
                assert pkg_command.wait_for_shutdown(timeout=5)
            assert pkg_command.exit_code == launch_testing.asserts.EXIT_OK
            assert launch_testing.tools.expect_output(
                expected_lines=[
                    'going to create a new package',
                    'package name: a_test_package',
                    'destination directory: ' + os.path.realpath(tmpdir),
                    'package format: 3',
                    'version: 0.0.0',
                    'description: A test package dummy description',
                    "maintainer: ['Nobody <nobody@nowhere.com>']",
                    "licenses: ['Apache License 2.0']",
                    'build type: ament_cmake',
                    "dependencies: ['ros2pkg']",
                    'node_name: test_node',
                    'library_name: test_library',
                    'creating folder ' + os.path.join('.', 'a_test_package'),
                    'creating ' + os.path.join('.', 'a_test_package', 'package.xml'),
                    'creating source and include folder',
                    'creating folder ' + os.path.join('.', 'a_test_package', 'src'),
                    'creating folder ' + os.path.join(
                        '.', 'a_test_package', 'include', 'a_test_package'
                    ),
                    'creating ' + os.path.join('.', 'a_test_package', 'CMakeLists.txt'),
                    'creating ' + os.path.join(
                        '.', 'a_test_package', 'src', 'test_node.cpp'
                    ),
                    'creating ' + os.path.join(
                        '.', 'a_test_package', 'include', 'a_test_package', 'test_library.hpp'
                    ),
                    'creating ' + os.path.join(
                        '.', 'a_test_package', 'src', 'test_library.cpp'
                    ),
                    'creating ' + os.path.join(
                        '.', 'a_test_package', 'include', 'a_test_package', 'visibility_control.h'
                    ),
                ],
                text=pkg_command.output,
                strict=True
            )
            # Check layout
            assert os.path.isdir(os.path.join(tmpdir, 'a_test_package'))
            assert os.path.isfile(os.path.join(tmpdir, 'a_test_package', 'package.xml'))
            assert os.path.isfile(os.path.join(tmpdir, 'a_test_package', 'CMakeLists.txt'))
            assert os.path.isfile(
                os.path.join(tmpdir, 'a_test_package', 'src', 'test_node.cpp')
            )
            assert os.path.isfile(
                os.path.join(tmpdir, 'a_test_package', 'src', 'test_library.cpp')
            )
            assert os.path.isfile(os.path.join(
                tmpdir, 'a_test_package', 'include', 'a_test_package', 'test_library.hpp'
            ))
            assert os.path.isfile(os.path.join(
                tmpdir, 'a_test_package', 'include', 'a_test_package', 'visibility_control.h'
            ))
            # Check package.xml
            tree = ET.parse(os.path.join(tmpdir, 'a_test_package', 'package.xml'))
            root = tree.getroot()
            assert root.tag == 'package'
            assert root.attrib['format'] == '3'
            assert root.find('name').text == 'a_test_package'
            assert root.find('description').text == 'A test package dummy description'
            assert root.find('maintainer').text == 'Nobody'
            assert root.find('maintainer').attrib['email'] == 'nobody@nowhere.com'
            assert root.find('license').text == 'Apache License 2.0'
            assert root.find('depend').text == 'ros2pkg'
            assert root.find('.//build_type').text == 'ament_cmake'
