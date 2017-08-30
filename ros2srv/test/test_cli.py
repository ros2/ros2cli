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
    packages_cmd = ['ros2', 'srv', 'packages']
    packages_result = subprocess.run(packages_cmd, stdout=subprocess.PIPE, check=True)
    package_names = packages_result.stdout.decode().splitlines()

    # explicit dependencies of this package will for sure be available
    assert 'std_srvs' in package_names

    count = 0
    for package_name in package_names:
        package_cmd = ['ros2', 'srv', 'package', package_name]
        package_result = subprocess.run(
            package_cmd, stdout=subprocess.PIPE, check=True)
        service_types = package_result.stdout.decode().splitlines()
        assert all(t.startswith(package_name + '/') for t in service_types)
        count += len(service_types)

        if package_name != 'std_srvs':
            continue
        for service_name in [t[len(package_name) + 1:] for t in service_types]:
            show_cmd = [
                'ros2', 'srv', 'show', package_name + '/' + service_name]
            show_result = subprocess.run(
                show_cmd, stdout=subprocess.PIPE, check=True)
            if service_name == 'Empty':
                assert show_result.stdout.rstrip() == b'---'

    list_cmd = ['ros2', 'srv', 'list']
    list_result = subprocess.run(list_cmd, stdout=subprocess.PIPE, check=True)
    service_types = list_result.stdout.decode().splitlines()
    assert len(service_types) == count

    package_cmd = ['ros2', 'srv', 'package', '_not_existing_package_name']
    package_result = subprocess.run(
        package_cmd, stdout=subprocess.PIPE)
    assert package_result.returncode

    show_cmd = ['ros2', 'srv', 'show', 'std_srvs/_not_existing_service_name']
    show_result = subprocess.run(
        show_cmd, stdout=subprocess.PIPE)
    assert show_result.returncode
