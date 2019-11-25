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
from ros2cli.node.strategy import DirectNode


def test_get_action_clients_and_servers():
    with DirectNode(None) as node:
        clients, servers = get_action_clients_and_servers(
            node=node,
            action_name='/test_action_name',
        )
    assert len(clients) == 0
    assert len(servers) == 0


def test_get_action_names():
    with DirectNode(None) as node:
        get_action_names(node=node)
