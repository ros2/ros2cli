# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import argparse

from ros2param.verb.list import ListVerb


def _parse(argv):
    parser = argparse.ArgumentParser()
    ListVerb().add_arguments(parser, 'ros2 param list')
    return parser.parse_args(argv)


def test_list_timeout_alias():
    assert _parse(['--timeout', '3.5']).service_timeout == 3.5


def test_list_service_timeout_still_works():
    assert _parse(['--service-timeout', '3.5']).service_timeout == 3.5
