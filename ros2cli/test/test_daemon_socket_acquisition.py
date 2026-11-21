# Copyright 2026 Sylvester Kaczmarek
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

import errno
import socket
from unittest.mock import Mock
from unittest.mock import patch

import pytest
import ros2cli.node.daemon as daemon_node


def _address_in_use_error():
    return socket.error(errno.EADDRINUSE, 'address already in use')


def test_free_address_does_not_probe_or_wait():
    expected_server = Mock()
    with patch.object(
        daemon_node.daemon, 'make_xmlrpc_server', return_value=expected_server
    ), patch.object(daemon_node, 'is_daemon_running') as probe, patch.object(
        daemon_node, 'wait_for'
    ) as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], 5.0)
    assert server is expected_server
    probe.assert_not_called()
    wait_for.assert_not_called()


def test_busy_address_with_running_daemon_is_not_retried():
    with patch.object(
        daemon_node.daemon, 'make_xmlrpc_server', side_effect=_address_in_use_error()
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=True
    ) as probe, patch.object(daemon_node.os, 'name', 'nt'), patch.object(
        daemon_node, 'wait_for'
    ) as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], 5.0)
    assert server is None
    probe.assert_called_once_with([], timeout=0.2)
    wait_for.assert_not_called()


def test_busy_windows_shutdown_tail_uses_remaining_grace_period():
    expected_server = Mock()
    with patch.object(
        daemon_node.daemon, 'make_xmlrpc_server',
        side_effect=[_address_in_use_error(), expected_server]
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=False
    ) as probe, patch.object(daemon_node.os, 'name', 'nt'), patch.object(
        daemon_node.time, 'monotonic', side_effect=[10.0, 10.15]
    ), patch.object(daemon_node, 'wait_for', return_value=True) as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], 5.0)
    assert server is expected_server
    probe.assert_called_once_with([], timeout=0.2)
    wait_for.assert_called_once()
    predicate, remaining = wait_for.call_args.args
    assert predicate is daemon_node._is_daemon_address_free
    assert remaining == pytest.approx(0.85)


def test_foreign_windows_port_owner_cannot_consume_full_spawn_timeout():
    with patch.object(
        daemon_node.daemon, 'make_xmlrpc_server', side_effect=_address_in_use_error()
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=False
    ), patch.object(daemon_node.os, 'name', 'nt'), patch.object(
        daemon_node.time, 'monotonic', side_effect=[10.0, 10.2]
    ), patch.object(daemon_node, 'wait_for', return_value=False) as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], -1.0)
    assert server is None
    predicate, remaining = wait_for.call_args.args
    assert predicate is daemon_node._is_daemon_address_free
    assert remaining == pytest.approx(0.8)


def test_short_caller_timeout_bounds_both_probe_and_release_wait():
    with patch.object(
        daemon_node.daemon, 'make_xmlrpc_server', side_effect=_address_in_use_error()
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=False
    ) as probe, patch.object(daemon_node.os, 'name', 'nt'), patch.object(
        daemon_node.time, 'monotonic', side_effect=[10.0, 10.02]
    ), patch.object(daemon_node, 'wait_for', return_value=False) as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], 0.05)
    assert server is None
    probe.assert_called_once_with([], timeout=0.05)
    assert wait_for.call_args.args[1] == pytest.approx(0.03)


def test_exhausted_probe_budget_never_becomes_an_indefinite_wait():
    with patch.object(
        daemon_node.daemon, 'make_xmlrpc_server', side_effect=_address_in_use_error()
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=False
    ), patch.object(daemon_node.os, 'name', 'nt'), patch.object(
        daemon_node.time, 'monotonic', side_effect=[10.0, 11.0]
    ), patch.object(daemon_node, 'wait_for') as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], -1.0)
    assert server is None
    wait_for.assert_not_called()


@pytest.mark.parametrize('platform, timeout', [('posix', 5.0), ('nt', None)])
def test_non_retry_paths_do_not_probe_an_unrelated_listener(platform, timeout):
    with patch.object(
        daemon_node.daemon, 'make_xmlrpc_server', side_effect=_address_in_use_error()
    ), patch.object(daemon_node, 'is_daemon_running') as probe, patch.object(
        daemon_node.os, 'name', platform
    ), patch.object(daemon_node, 'wait_for') as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], timeout)
    assert server is None
    probe.assert_not_called()
    wait_for.assert_not_called()


def test_losing_the_second_bind_race_returns_no_server():
    with patch.object(
        daemon_node.daemon, 'make_xmlrpc_server', side_effect=_address_in_use_error()
    ) as make_server, patch.object(
        daemon_node, 'is_daemon_running', return_value=False
    ), patch.object(daemon_node.os, 'name', 'nt'), patch.object(
        daemon_node, 'wait_for', return_value=True
    ):
        server = daemon_node._make_xmlrpc_server_when_available([], 5.0)
    assert server is None
    assert make_server.call_count == 2


def test_unexpected_bind_error_is_preserved():
    error = OSError(errno.EACCES, 'permission denied')
    with patch.object(daemon_node.daemon, 'make_xmlrpc_server', side_effect=error):
        with pytest.raises(OSError) as caught:
            daemon_node._make_xmlrpc_server_when_available([], 5.0)
    assert caught.value is error


def test_probe_timeout_does_not_change_global_socket_defaults():
    original_timeout = socket.getdefaulttimeout()
    transport = daemon_node._TimeoutTransport(0.05)
    try:
        connection = transport.make_connection('127.0.0.1:1')
        assert connection.timeout == 0.05
        assert transport.make_connection('127.0.0.1:1') is connection
        assert socket.getdefaulttimeout() == original_timeout
    finally:
        transport.close()
