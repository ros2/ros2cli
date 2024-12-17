# Copyright 2020 Open Source Robotics Foundation, Inc.
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

from ros2cli.node.daemon import is_daemon_running
from ros2cli.node.daemon import shutdown_daemon
from ros2cli.node.daemon import spawn_daemon
from ros2cli.node.strategy import NodeStrategy


@pytest.fixture
def enforce_no_daemon_is_running():
    if is_daemon_running(args=[], timeout=5.0):
        assert shutdown_daemon(args=[], timeout=5.0)
    yield


@pytest.fixture
def enforce_daemon_is_running():
    if not is_daemon_running(args=[], timeout=5.0):
        assert spawn_daemon(args=[], timeout=5.0)
    yield


def test_with_daemon_running(enforce_daemon_is_running):
    with NodeStrategy(args=[]) as node:
        assert node._daemon_node is not None
        assert node._direct_node is None
        # force direct node instantiation
        direct = node.direct_node
        assert direct is not None


def test_with_daemon_spawn(enforce_no_daemon_is_running):
    with NodeStrategy(args=[]) as node:
        assert node._daemon_node is None
        assert node._direct_node is not None
    # Daemon should be spawned by NodeStrategy call above
    with NodeStrategy(args=[]) as node:
        assert node._daemon_node is not None
        assert node._direct_node is None


def test_with_no_daemon_running(enforce_no_daemon_is_running):
    with NodeStrategy(args=[]) as node:
        assert node._daemon_node is None
        assert node._direct_node is not None


def test_enforce_no_daemon(enforce_daemon_is_running):
    args = argparse.Namespace(no_daemon=True)
    with NodeStrategy(args=args) as node:
        assert node._daemon_node is None
        assert node._direct_node is not None
