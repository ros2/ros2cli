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

import ros2cli.node.daemon as daemon_node


def _address_in_use_error():
    return socket.error(errno.EADDRINUSE, 'address already in use')


def test_busy_address_with_running_daemon_is_not_retried():
    with patch.object(
        daemon_node.daemon,
        'make_xmlrpc_server',
        side_effect=_address_in_use_error(),
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=True
    ), patch.object(daemon_node, 'wait_for') as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], 5.0)

    assert server is None
    wait_for.assert_not_called()


def test_busy_windows_shutdown_tail_is_retried_after_address_becomes_free():
    expected_server = Mock()
    with patch.object(
        daemon_node.daemon,
        'make_xmlrpc_server',
        side_effect=[_address_in_use_error(), expected_server],
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=False
    ), patch.object(daemon_node.os, 'name', 'nt'), patch.object(
        daemon_node, 'wait_for', return_value=True
    ) as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], 5.0)

    assert server is expected_server
    wait_for.assert_called_once_with(
        daemon_node._is_daemon_address_free,
        daemon_node._DAEMON_SOCKET_RELEASE_GRACE_PERIOD,
    )


def test_foreign_windows_port_owner_cannot_consume_full_spawn_timeout():
    with patch.object(
        daemon_node.daemon,
        'make_xmlrpc_server',
        side_effect=_address_in_use_error(),
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=False
    ), patch.object(daemon_node.os, 'name', 'nt'), patch.object(
        daemon_node, 'wait_for', return_value=False
    ) as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], -1.0)

    assert server is None
    wait_for.assert_called_once_with(
        daemon_node._is_daemon_address_free,
        daemon_node._DAEMON_SOCKET_RELEASE_GRACE_PERIOD,
    )


def test_busy_non_windows_address_is_not_retried():
    with patch.object(
        daemon_node.daemon,
        'make_xmlrpc_server',
        side_effect=_address_in_use_error(),
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=False
    ), patch.object(daemon_node.os, 'name', 'posix'), patch.object(
        daemon_node, 'wait_for'
    ) as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], 5.0)

    assert server is None
    wait_for.assert_not_called()


def test_busy_address_without_timeout_preserves_non_blocking_behavior():
    with patch.object(
        daemon_node.daemon,
        'make_xmlrpc_server',
        side_effect=_address_in_use_error(),
    ), patch.object(
        daemon_node, 'is_daemon_running', return_value=False
    ), patch.object(daemon_node, 'wait_for') as wait_for:
        server = daemon_node._make_xmlrpc_server_when_available([], None)

    assert server is None
    wait_for.assert_not_called()
