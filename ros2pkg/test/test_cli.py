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
import subprocess


def test_cli():
    list_cmd = ['ros2', 'pkg', 'list']
    list_result = subprocess.run(list_cmd, stdout=subprocess.PIPE, check=True)
    package_names = list_result.stdout.decode().splitlines()

    # explicit dependencies of this package will for sure be available
    assert 'ros2cli' in package_names

    prefix_cmd = ['ros2', 'pkg', 'prefix', 'ros2cli']
    prefix_result = subprocess.run(
        prefix_cmd, stdout=subprocess.PIPE, check=True)
    prefix_path = prefix_result.stdout.decode().splitlines()
    assert len(prefix_path) == 1
    assert os.path.isdir(prefix_path[0])

    prefix_cmd = ['ros2', 'pkg', 'prefix', '_not_existing_package_name']
    prefix_result = subprocess.run(
        prefix_cmd, stdout=subprocess.PIPE)
    assert prefix_result.returncode
