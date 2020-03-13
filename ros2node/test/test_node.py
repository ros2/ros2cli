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

from ros2node.api import has_duplicates
from ros2node.api import parse_node_name


def test_parse_node_name():
    """Test parse_node_name api function."""
    # The input of parse_node_name is the full_name without the initial '/'
    name = parse_node_name('/talker')
    assert name.full_name == '/talker'
    assert name.namespace == '/'
    assert name.name == 'talker'
    name = parse_node_name('/ns/talker')
    assert name.full_name == '/ns/talker'
    assert name.namespace == '/ns'
    assert name.name == 'talker'
    name = parse_node_name('talker')
    assert name.full_name == '/talker'
    assert name.namespace == '/'
    assert name.name == 'talker'
    name = parse_node_name('ns/talker')
    assert name.full_name == '/ns/talker'
    assert name.namespace == '/ns'
    assert name.name == 'talker'


def test_has_duplicates():
    assert not has_duplicates([])
    assert not has_duplicates(['ns_foo/foo', 'ns_foo/bar'])
    assert has_duplicates(['ns_foo/foo'] * 2)
    assert has_duplicates(['ns_foo/foo'] * 3)
    assert has_duplicates(['ns_foo/foo', 'ns_foo/foo', 'ns_foo/bar'])
