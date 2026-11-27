# Copyright 2026 Dominik Authaler
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
from unittest.mock import Mock

from builtin_interfaces.msg import Time as TimeMsg

import pytest

from rclpy.clock import ClockType
from rclpy.time import Time

from ros2topic.verb import delay


@pytest.mark.parametrize(('backends', 'expected_backends'), [
    (None, None),
    ('', None),
    (' \t ', None),
    (', ,', None),
    ('cpu', None),
    (' CPU , cpu, ', None),
    ('any', 'any'),
    (' ANY, any, ', 'any'),
    ('BACKEND_A', 'backend_a'),
    ('backend_a,cpu', 'backend_a,cpu'),
    (' BACKEND_A, cpu, BACKEND_A, , ', 'backend_a,cpu'),
    (' Backend_B, BACKEND_A ', 'backend_b,backend_a'),
])
def test_delay_buffer_backends(monkeypatch, backends, expected_backends):
    parser = argparse.ArgumentParser()
    parser.set_defaults(include_hidden_topics=False)
    delay.DelayVerb().add_arguments(parser, 'ros2 topic delay')
    arguments = ['/image']
    if backends is not None:
        arguments += ['--acceptable-buffer-backends', backends]
    args = parser.parse_args(arguments)

    # mocking a scenario with a fixed message stamp of 1.5s, whereas "now" is at 2.0s
    #   --> expected delay 0.5s
    node = Mock()
    node.get_clock.return_value.now.return_value = Time(nanoseconds=2_000_000_000,
                                                        clock_type=ClockType.ROS_TIME)
    direct_node = Mock()
    direct_node.__enter__ = Mock(return_value=SimpleNamespace(node=node))
    direct_node.__exit__ = Mock(return_value=False)
    monkeypatch.setattr(delay, 'DirectNode', Mock(return_value=direct_node))
    monkeypatch.setattr(delay, 'get_msg_class', Mock(return_value=object))
    monkeypatch.setattr(delay, 'choose_qos', Mock())
    # skips the spin loop as our test feeds the mocked messages directly
    monkeypatch.setattr(delay.rclpy, 'ok', Mock(return_value=False))
    monkeypatch.setattr(delay.rclpy, 'spin_once', Mock())
    monkeypatch.setattr(delay.rclpy, 'shutdown', Mock())

    assert delay.main(args) == 0
    node.create_subscription.assert_called_once()
    call = node.create_subscription.call_args

    assert call.args[1] == '/image'
    # delay needs header.stamp --> no raw subscription possible, deserialization is always involved
    assert 'raw' not in call.kwargs
    if expected_backends is None:
        assert 'acceptable_buffer_backends' not in call.kwargs
    else:
        assert call.kwargs['acceptable_buffer_backends'] == expected_backends

    class Message:
        header = SimpleNamespace(stamp=TimeMsg(sec=1, nanosec=500_000_000))

        @property
        def data(self):
            raise AssertionError('delay accessed the buffer payload')

    callback = call.args[2]                # args: (msg_class, topic, callback, ...)
    callback(Message())                    # first message only initializes the window
    callback(Message())                    # second message records one delay sample
    mean, min_delta, max_delta, stddev, window = callback.__self__.get_delay()
    assert mean == pytest.approx(500_000_000)    # unit: nanoseconds
    assert stddev == 0                           # single sample --> no stddev computable yet
    assert min_delta == max_delta == 500_000_000
    assert window == 1


def test_delay_buffer_backends_requires_value():
    # checks that bare --acceptable-buffer-backends (no value) is a usage error
    parser = argparse.ArgumentParser()
    delay.DelayVerb().add_arguments(parser, 'ros2 topic delay')
    with pytest.raises(SystemExit) as exc:
        parser.parse_args(['/image', '--acceptable-buffer-backends'])
    assert exc.value.code == 2
