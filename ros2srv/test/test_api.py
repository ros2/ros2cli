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

from ros2srv.api import get_all_service_types
from ros2srv.api import get_service_path
from ros2srv.api import get_service_types


def test_api():
    all_service_types = get_all_service_types()

    # explicit dependencies of this package will for sure be available
    assert 'std_srvs' in all_service_types.keys()

    for package_name, service_types in all_service_types.items():
        assert isinstance(service_types, Iterable)

        service_types2 = get_service_types(package_name)
        assert set(service_types) == set(service_types2)

        if package_name != 'std_srvs':
            continue
        for service_name in service_types:
            service_path = get_service_path(package_name, service_name)
            assert os.path.isfile(service_path)

    # check known package name
    get_service_types('std_srvs')
    service_path = get_service_path('std_srvs', 'Empty')
    assert os.path.isfile(service_path)

    # check not existing package name
    try:
        get_service_types('_not_existing_package_name')
        assert False
    except LookupError:
        pass

    # check package with doesn't have services
    service_names = get_service_types('std_msgs')
    assert len(service_names) == 0

    # check known package for not existing service name
    try:
        get_service_path('std_srvs', '_not_existing_service_name')
        assert False
    except LookupError:
        pass
