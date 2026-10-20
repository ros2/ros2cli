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


@functools.lru_cache(maxsize=None)
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


def test_api():
    sent_data = b'test_api'
    received_data = None

    def target():
        nonlocal received_data
        received_data, _ = receive(timeout=1.0)

    t = threading.Thread(target=target)
    t.start()
    time.sleep(0.1)
    send(sent_data)
    t.join()
    assert sent_data == received_data
