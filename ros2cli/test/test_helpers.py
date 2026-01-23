# Copyright 2026 Sony Corporation
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

import io
import os
from unittest.mock import patch

import ros2cli.helpers
from ros2cli.helpers import check_discovery_configuration


def test_check_discovery_configuration_off_without_peers():
    """Test warning is shown when ROS_AUTOMATIC_DISCOVERY_RANGE=OFF without ROS_STATIC_PEERS."""
    # Reset the module-level flag
    ros2cli.helpers._discovery_warning_shown = False

    env = {
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'OFF',
    }
    # Make sure ROS_STATIC_PEERS is not set
    env_without_peers = {k: v for k, v in os.environ.items() if k != 'ROS_STATIC_PEERS'}
    env_without_peers.update(env)

    # Capture stderr
    stderr_capture = io.StringIO()
    with patch.dict(os.environ, env_without_peers, clear=True):
        with patch('sys.stderr', stderr_capture):
            check_discovery_configuration()

    output = stderr_capture.getvalue()
    assert 'Warning: ROS_AUTOMATIC_DISCOVERY_RANGE=OFF' in output
    assert 'No discovery mechanism is available' in output
    assert 'ROS_STATIC_PEERS' in output
    assert 'LOCALHOST or SUBNET' in output


def test_check_discovery_configuration_off_with_peers():
    """Test no warning when ROS_AUTOMATIC_DISCOVERY_RANGE=OFF with ROS_STATIC_PEERS set."""
    # Reset the module-level flag
    ros2cli.helpers._discovery_warning_shown = False

    env = {
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'OFF',
        'ROS_STATIC_PEERS': '192.168.1.10;192.168.1.11',
    }

    # Capture stderr
    stderr_capture = io.StringIO()
    with patch.dict(os.environ, env, clear=True):
        with patch('sys.stderr', stderr_capture):
            check_discovery_configuration()

    output = stderr_capture.getvalue()
    assert output == '', 'Expected no warning output'


def test_check_discovery_configuration_off_with_empty_peers():
    """Test warning is shown when ROS_STATIC_PEERS is set but empty."""
    # Reset the module-level flag
    ros2cli.helpers._discovery_warning_shown = False

    env = {
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'OFF',
        'ROS_STATIC_PEERS': '',
    }

    # Capture stderr
    stderr_capture = io.StringIO()
    with patch.dict(os.environ, env, clear=True):
        with patch('sys.stderr', stderr_capture):
            check_discovery_configuration()

    output = stderr_capture.getvalue()
    assert 'Warning: ROS_AUTOMATIC_DISCOVERY_RANGE=OFF' in output


def test_check_discovery_configuration_not_set():
    """Test no warning when ROS_AUTOMATIC_DISCOVERY_RANGE is not set (default behavior)."""
    # Reset the module-level flag
    ros2cli.helpers._discovery_warning_shown = False

    # Make sure neither env var is set
    env_clean = {
        k: v for k, v in os.environ.items()
        if k not in ('ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS')
    }

    # Capture stderr
    stderr_capture = io.StringIO()
    with patch.dict(os.environ, env_clean, clear=True):
        with patch('sys.stderr', stderr_capture):
            check_discovery_configuration()

    output = stderr_capture.getvalue()
    assert output == '', 'Expected no warning output'
