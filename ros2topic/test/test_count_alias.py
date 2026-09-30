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

import pytest

from ros2topic.verb.find import FindVerb
from ros2topic.verb.list import ListVerb


@pytest.mark.parametrize('option', ['-c', '--count-topics', '--count'])
def test_find_count_alias(option):
    parser = argparse.ArgumentParser(allow_abbrev=False)
    FindVerb().add_arguments(parser, 'ros2 topic find')
    args = parser.parse_args(['std_msgs/msg/String', option])
    assert args.count_topics is True


@pytest.mark.parametrize('option', ['-c', '--count-topics', '--count'])
def test_list_count_alias(option):
    parser = argparse.ArgumentParser(allow_abbrev=False)
    ListVerb().add_arguments(parser, 'ros2 topic list')
    args = parser.parse_args([option])
    assert args.count_topics is True
