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

from collections import Iterable
import os

from ros2pkg.api import get_package_names
from ros2pkg.api import get_prefix_path
from ros2pkg.api import get_share_directory


def test_api():
    package_names = get_package_names()
    assert isinstance(package_names, Iterable)

    # explicit dependencies of this package will for sure be available
    known_package_names = (
        'ament_copyright', 'ament_flake8', 'ament_pep257', 'ros2cli')
    for known_package_name in known_package_names:
        assert known_package_name in package_names

        prefix_path = get_prefix_path(known_package_name)
        assert os.path.isdir(prefix_path)
        share_directory = get_share_directory(known_package_name)
        assert os.path.isdir(share_directory)
        assert 'share' in share_directory

    not_existing_package_name = '_not_existing_package_name'
    prefix_path = get_prefix_path(not_existing_package_name)
    assert prefix_path is None
    share_directory = get_share_directory(not_existing_package_name)
    assert share_directory is None
