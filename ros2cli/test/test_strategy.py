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
from types import SimpleNamespace

import pytest

import ros2cli.daemon as daemon
from ros2cli.node.daemon import DaemonNode
from ros2cli.node.daemon import is_daemon_running
from ros2cli.node.daemon import shutdown_daemon
from ros2cli.node.daemon import spawn_daemon
import ros2cli.node.strategy as strategy_module
from ros2cli.node.strategy import NodeStrategy


@pytest.fixture
def enforce_no_daemon_is_running():
    if is_daemon_running(args=[]):
        assert shutdown_daemon(args=[], timeout=5.0)
    yield


@pytest.fixture
def enforce_daemon_is_running():
    if not is_daemon_running(args=[]):
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


def test_daemon_configuration_matches_current_process():
    node = DaemonNode(args=[])
    node._methods = ['get_daemon_configuration']
    node._proxy = SimpleNamespace(
        get_daemon_configuration=daemon.get_daemon_configuration
    )

    assert node.configuration_matches


def test_daemon_configuration_rejects_different_rmw():
    configuration = daemon.get_daemon_configuration()
    configuration['rmw_implementation'] += '_different'
    node = DaemonNode(args=[])
    node._methods = ['get_daemon_configuration']
    node._proxy = SimpleNamespace(
        get_daemon_configuration=lambda: configuration
    )

    assert not node.configuration_matches


def test_daemon_configuration_rejects_unverifiable_daemon():
    node = DaemonNode(args=[])
    node._methods = []

    assert not node.configuration_matches


def test_strategy_falls_back_for_mismatched_daemon(monkeypatch, capsys):
    daemon_node = SimpleNamespace(
        connected=True,
        configuration_matches=False,
    )
    direct_node = object()

    monkeypatch.setattr(strategy_module, 'check_discovery_configuration', lambda: None)
    monkeypatch.setattr(strategy_module, 'is_daemon_running', lambda args: True)
    monkeypatch.setattr(strategy_module, 'DaemonNode', lambda args: daemon_node)
    monkeypatch.setattr(
        strategy_module,
        'DirectNode',
        lambda args, node_name=None: direct_node,
    )

    strategy = NodeStrategy(args=[])

    assert strategy._daemon_node is None
    assert strategy._direct_node is direct_node
    assert 'Falling back to direct discovery' in capsys.readouterr().err
