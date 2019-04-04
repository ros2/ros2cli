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

from ros2action.api import get_action_clients_and_servers
from ros2action.api import get_action_names
from ros2action.api import get_action_path
from ros2action.api import get_action_types
from ros2cli.node.strategy import DirectNode

g_node = DirectNode(None)


def test_get_action_clients_and_servers():
    clients, servers = get_action_clients_and_servers(
        node=g_node,
        action_name='/test_action_name',
    )
    assert len(clients) == 0
    assert len(servers) == 0


def test_get_action_names():
    get_action_names(node=g_node)


def test_get_action_path():
    action_path = get_action_path('test_msgs', 'Fibonacci')
    assert os.path.isfile(action_path)

    with pytest.raises(LookupError):
        get_action_path('_not_a_real_package_name', 'Fibonacci')


def test_get_action_types():
    action_types = get_action_types('test_msgs')
    # Expect only strings
    for t in action_types:
        assert isinstance(t, str)
    # Explicit dependencies of this package should be available
    assert 'Fibonacci' in action_types
    assert 'NestedMessage' in action_types
    # Some things that should not be in the list
    assert '' not in action_types
    assert 'test_msgs' not in action_types
    assert 'NotAnActionMessage' not in action_types
