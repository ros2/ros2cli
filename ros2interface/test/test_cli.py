# Copyright 2019 Open Source Robotics Foundation, Inc.
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
    packages_cmd = ['ros2', 'interface', 'packages']
    packages_result = subprocess.run(packages_cmd, stdout=subprocess.PIPE, check=True)
    package_names = packages_result.stdout.decode().splitlines()

    # explicit dependencies of this package will for sure be available
    assert 'std_msgs' in package_names

    count = 0
    for package_name in package_names:
        package_cmd = ['ros2', 'interface', 'package', package_name]
        package_result = subprocess.run(
            package_cmd, stdout=subprocess.PIPE, check=True)
        message_types = package_result.stdout.decode().splitlines()
        assert all(t.startswith(package_name + '/') for t in message_types)
        count += len(message_types)

        if package_name != 'std_msgs':
            continue
        for message_name in [t[len(package_name) + 1:] for t in message_types]:
            show_cmd = [
                'ros2', 'interface', 'show', package_name + '/' + message_name]
            show_result = subprocess.run(
                show_cmd, stdout=subprocess.PIPE, check=True)
            if message_name == 'String':
                assert show_result.stdout.rstrip() == b'string data'

    package_cmd = ['ros2', 'interface', 'package', '_not_existing_package_name']
    package_result = subprocess.run(
        package_cmd, stdout=subprocess.PIPE)
    assert package_result.returncode

    show_cmd = ['ros2', 'interface', 'show', 'std_msgs/_not_existing_message_name']
    show_result = subprocess.run(
        show_cmd, stdout=subprocess.PIPE)
    assert show_result.returncode
