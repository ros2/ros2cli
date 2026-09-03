# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import random
import re
import subprocess
import sys
import threading
import time

import pytest

from ros2multicast.api import receive
from ros2multicast.api import send

_MULTICAST_ADDRESS = re.compile(r'\b2(?:2[4-9]|3[0-9])(?:\.\d{1,3}){3}\b')
_UDP_ENDPOINT = re.compile(r'^\s*(?:UDP|udp)\s+\S', re.MULTILINE)


def _capture(argv):
    try:
        return subprocess.run(argv, capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.SubprocessError):
        return ''


def _network_state():
    """
    Summarize the machine-wide network resources these tests draw on.

    Both the UDP port pool and the multicast membership table are shared by every process
    on the machine.  Once either is exhausted, binding and joining fail for everything on
    the box, which is easy to mistake for a defect in the code under test.
    See https://github.com/ros2/ros2cli/issues/1141.
    """
    if sys.platform.startswith('win'):
        joins = _capture(['netsh', 'interface', 'ipv4', 'show', 'joins'])
        endpoints = _capture(['netstat', '-ano', '-p', 'UDP'])
        port_range = '+'.join(re.findall(
            r':\s*(\d+)',
            _capture(['netsh', 'int', 'ipv4', 'show', 'dynamicport', 'udp'])))
    else:
        joins = _capture(['netstat', '--groups', '--numeric'])
        endpoints = _capture(['ss', '-uan'])
        port_range = _capture(['sysctl', '-n', 'net.ipv4.ip_local_port_range']).strip()
    return (
        f'{len(_MULTICAST_ADDRESS.findall(joins))} multicast memberships, '
        f'{len(_UDP_ENDPOINT.findall(endpoints))} udp endpoints open, '
        f'dynamic port range: {port_range or "unknown"}'
    )


def _reraise(error):
    """Re-raise a socket error, first recording the machine state that may have caused it."""
    print(
        f'ros2multicast socket operation failed ({error}); machine network state: '
        f'{_network_state()}',
        file=sys.stderr)
    raise error


def _send_receive(sent_data, rx_kwargs, tx_kwargs):
    received_data = None
    rx_error = None

    def target():
        nonlocal received_data, rx_error
        try:
            received_data, _ = receive(**rx_kwargs)
        except TimeoutError:
            pass
        except Exception as e:  # noqa: B902
            # Stash it for the calling thread. Left unhandled here, pytest turns it into
            # an unhandled-thread-exception warning, so the test either fails with a
            # misleading assertion or passes when it should not.
            rx_error = e

    t = threading.Thread(target=target)
    t.start()
    time.sleep(0.1)
    try:
        send(sent_data, **tx_kwargs)
    except OSError as e:
        _reraise(e)
    finally:
        t.join()
    if rx_error is not None:
        _reraise(rx_error)
    return received_data


def test_api():
    sent_data = b'test_api'

    rx_kwargs = {'timeout': 0.2}
    tx_kwargs = {}

    assert sent_data == _send_receive(sent_data, rx_kwargs, tx_kwargs)


def test_group_and_port():
    sent_data = b'test_api'

    r1 = random.randrange(0, 255)
    r2 = random.randrange(0, 255)
    r3 = random.randrange(1, 255)
    port = random.randrange(16000, 18000)

    group = f'234.{r1}.{r2}.{r3}'

    tx_kwargs = {'group': group, 'port': port}
    rx_kwargs = {'timeout': 1.0}
    rx_kwargs.update(tx_kwargs)

    assert sent_data == _send_receive(sent_data, rx_kwargs, tx_kwargs)


def test_group_mismatch():
    sent_data = b'test_api'

    r1 = random.randrange(0, 255)
    r2 = random.randrange(0, 255)
    r3 = random.randrange(1, 255)
    port = random.randrange(16000, 18000)

    group1 = f'234.{r1}.{r2}.{r3}'
    group2 = f'235.{r1}.{r2}.{r3}'

    tx_kwargs = {'group': group1, 'port': port}
    rx_kwargs = {'group': group2, 'port': port, 'timeout': 1.0}

    try:
        assert _send_receive(sent_data, rx_kwargs, tx_kwargs) is None
    except OSError as e:
        if sys.platform.startswith('win') and 10051 == e.winerror:
            # TODO(sloretz) understand why this test fails this way in CI
            # "A socket operation was attempted to an unreachable network"
            pytest.skip('Unknown why this OSError occurs on Windows')
        raise


def test_port_mismatch():
    sent_data = b'test_api'

    r1 = random.randrange(0, 255)
    r2 = random.randrange(0, 255)
    r3 = random.randrange(1, 255)
    port1 = random.randrange(16000, 18000)
    port2 = port1 + 1

    group = f'234.{r1}.{r2}.{r3}'

    tx_kwargs = {'group': group, 'port': port1}
    rx_kwargs = {'group': group, 'port': port2, 'timeout': 1.0}

    assert _send_receive(sent_data, rx_kwargs, tx_kwargs) is None
