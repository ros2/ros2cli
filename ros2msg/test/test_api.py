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

from ros2msg.api import get_all_message_types
from ros2msg.api import get_message_path
from ros2msg.api import get_message_types


def test_api():
    all_message_types = get_all_message_types()

    # explicit dependencies of this package will for sure be available
    assert 'std_msgs' in all_message_types.keys()

    for package_name, message_types in all_message_types.items():
        assert isinstance(message_types, Iterable)

        message_types2 = get_message_types(package_name)
        assert set(message_types) == set(message_types2)

        if package_name != 'std_msgs':
            continue
        for message_name in message_types:
            message_path = get_message_path(package_name, message_name)
            assert os.path.isfile(message_path)

    # check known package name
    get_message_types('std_msgs')
    message_path = get_message_path('std_msgs', 'Empty')
    assert os.path.isfile(message_path)

    # check not existing package name
    try:
        get_message_types('_not_existing_package_name')
        assert False
    except LookupError:
        pass

    # check package with doesn't have messages
    message_names = get_message_types('std_srvs')
    assert len(message_names) == 0

    # check known package for not existing message name
    try:
        get_message_path('std_msgs', '_not_existing_message_name')
        assert False
    except LookupError:
        pass
