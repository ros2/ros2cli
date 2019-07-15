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
import re
import sys

sys.path.append(os.path.dirname(__file__))

from cli_test_configuration import CLITestConfiguration  # noqa


some_services_from_std_srvs = [
    'std_srvs/srv/Empty',
    'std_srvs/srv/SetBool',
    'std_srvs/srv/Trigger',
]


def get_test_configurations(*args, **kwargs):
    return [
        CLITestConfiguration(
            command='srv',
            arguments=['list'],
            expected_output=(
                lambda lines: (
                    all(srv in lines for srv in some_services_from_std_srvs) and
                    all(re.match(r'.*/srv/.*', line) is not None for line in lines)
                )
            )
        ),
        CLITestConfiguration(
            command='srv',
            arguments=['package', 'std_srvs'],
            expected_output=(
                lambda lines: (
                    all(srv in lines for srv in some_services_from_std_srvs) and
                    all(re.match(r'std_srvs/srv/.*', line) is not None for line in lines)
                )
            )
        ),
        CLITestConfiguration(
            command='srv',
            arguments=['package', 'not_a_package'],
            expected_output=['Unknown package name'],
            exit_codes=[1]
        ),
        CLITestConfiguration(
            command='srv',
            arguments=['packages'],
            expected_output=lambda lines: 'std_srvs' in lines
        ),
        CLITestConfiguration(
            command='srv',
            arguments=['show', 'std_srvs/srv/SetBool'],
            expected_output=['bool data', '---', 'bool success', 'string message']
        ),
        CLITestConfiguration(
            command='srv',
            arguments=['show', 'std_srvs/srv/Trigger'],
            expected_output=['---', 'bool success', 'string message']
        ),
        CLITestConfiguration(
            command='srv',
            arguments=['show', 'std_srvs/srv/NotAServiceType'],
            expected_output=['Unknown service name'],
            exit_codes=[1]
        ),
        CLITestConfiguration(
            command='srv',
            arguments=['show', 'not_a_package/srv/Empty'],
            expected_output=['Unknown package name'],
            exit_codes=[1]
        ),
        CLITestConfiguration(
            command='srv',
            arguments=['show', 'not_a_service_type'],
            expected_output=['The passed service type is invalid'],
            exit_codes=[1]
        ),
    ]
