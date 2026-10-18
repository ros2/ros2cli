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

import functools
import logging
import random
import sys
import threading
import time

import pytest

from ros2multicast.api import receive
from ros2multicast.api import send

# On some Windows agents (Windows Server 2025 containers) sending to any multicast group fails
# with WSAENOBUFS (10055), regardless of resource usage or interface.
# TODO(ros2cli#1141): remove once multicast works on those agents.
_WSAENOBUFS = 10055
_ISSUE = 'https://github.com/ros2/ros2cli/issues/1141'


@functools.cache
def _multicast_known_broken():
    """Return a reason if multicast sending is known to be broken here, else an empty string."""
    if not sys.platform.startswith('win'):
        return ''
    try:
        send(b'probe')
    except OSError as e:
        if getattr(e, 'winerror', None) == _WSAENOBUFS:
            reason = f'multicast send fails with WinError {_WSAENOBUFS} on this machine ({_ISSUE})'
            logging.getLogger(__name__).warning('Skipping multicast tests: %s', reason)
            return reason
        # Any other failure is left for the tests themselves to report.
    return ''


@pytest.fixture(autouse=True)
def skip_if_multicast_known_broken():
    reason = _multicast_known_broken()
    if reason:
        pytest.skip(reason)


def _send_receive(sent_data, rx_kwargs, tx_kwargs):
    received_data = None

    def target():
        nonlocal received_data
        try:
            received_data, _ = receive(**rx_kwargs)
        except TimeoutError:
            pass

    t = threading.Thread(target=target)
    t.start()
    time.sleep(0.1)
    send(sent_data, **tx_kwargs)
    t.join()
    return received_data


def test_api():
    sent_data = b'test_api'

    rx_kwargs = {'timeout': 1.0}
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
        if sys.platform.startswith('win'):
            if 10051 == e.winerror:
                # TODO(sloretz) understand why this test fails this way in CI
                # "A socket operation was attempted to an unreachable network"
                pytest.skip('Unknown why this OSError occurs on Windows')
        else:
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
