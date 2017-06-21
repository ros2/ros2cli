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

import subprocess


def test_cli():
    packages_cmd = ['ros2', 'msg', 'packages']
    packages_result = subprocess.run(packages_cmd, stdout=subprocess.PIPE, check=True)
    package_names = packages_result.stdout.decode().splitlines()

    # explicit dependencies of this package will for sure be available
    assert 'std_msgs' in package_names

    count = 0
    for package_name in package_names:
        package_cmd = ['ros2', 'msg', 'package', package_name]
        package_result = subprocess.run(
            package_cmd, stdout=subprocess.PIPE, check=True)
        message_types = package_result.stdout.decode().splitlines()
        assert all([t.startswith(package_name + '/') for t in message_types])
        count += len(message_types)

        if package_name != 'std_msgs':
            continue
        for message_name in [t[len(package_name) + 1:] for t in message_types]:
            show_cmd = [
                'ros2', 'msg', 'show', package_name + '/' + message_name]
            show_result = subprocess.run(
                show_cmd, stdout=subprocess.PIPE, check=True)
            if message_name == 'String':
                assert show_result.stdout.rstrip() == b'string data'

    list_cmd = ['ros2', 'msg', 'list']
    list_result = subprocess.run(list_cmd, stdout=subprocess.PIPE, check=True)
    message_types = list_result.stdout.decode().splitlines()
    assert len(message_types) == count

    package_cmd = ['ros2', 'msg', 'package', '_not_existing_package_name']
    package_result = subprocess.run(
        package_cmd, stdout=subprocess.PIPE)
    assert package_result.returncode

    show_cmd = ['ros2', 'msg', 'show', 'std_msgs/_not_existing_message_name']
    show_result = subprocess.run(
        show_cmd, stdout=subprocess.PIPE)
    assert show_result.returncode
