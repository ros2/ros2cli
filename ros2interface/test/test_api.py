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

import os

import pytest

from ros2interface.api import get_all_interface_packages
from ros2interface.api import get_interface
from ros2interface.api import get_interface_path


def test_get_interface():
    # check nonexistent package name
    with pytest.raises(LookupError):
        get_interface('_not_existing_package_name')

    # check existing package name
    interface_names = get_interface('std_srvs')
    assert len(interface_names) == 3


def test_get_interface_path():
    # check for existing path
    path = ['std_msgs', 'msg', 'Empty']
    interface_path = get_interface_path(path)
    assert os.path.isfile(interface_path)

    # check for nonexistent path
    with pytest.raises(LookupError):
        path = ['std_msgs', 'msg', 'InvalidPath']
        get_interface_path(path)


def test_get_all_interface_pacakges():
    # check all packages being found
    interface_packages = get_all_interface_packages()
    assert 'std_msgs' in interface_packages
    assert 'std_srvs' in interface_packages
    assert len(interface_packages) >= 1
