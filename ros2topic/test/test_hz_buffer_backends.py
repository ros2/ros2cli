# Copyright 2026 NVIDIA CORPORATION
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

import pytest

from ros2topic.verb import hz


@pytest.mark.parametrize('backends', [None, 'any', 'cpu', 'cuda,cpu'])
@pytest.mark.parametrize('filtered', [False, True])
@pytest.mark.parametrize('all_topics', [False, True])
def test_hz_buffer_backends(monkeypatch, backends, filtered, all_topics):
    parser = argparse.ArgumentParser()
    parser.set_defaults(include_hidden_topics=False)
    hz.HzVerb().add_arguments(parser, 'ros2 topic hz')
    topics = ['/image', '/other_image']
    arguments = ['--all'] if all_topics else topics.copy()
    if backends is not None:
        arguments += ['--acceptable-buffer-backends', backends]
    if filtered:
        arguments += ['--filter', 'm.header.frame_id == "camera"']
    args = parser.parse_args(arguments)

    node = Mock()
    node.get_clock.return_value.now.side_effect = [
        SimpleNamespace(nanoseconds=t) for t in [1_000_000_000, 2_000_000_000] * 2
    ]
    direct_node = Mock()
    direct_node.__enter__ = Mock(return_value=SimpleNamespace(node=node))
    direct_node.__exit__ = Mock(return_value=False)
    monkeypatch.setattr(hz, 'DirectNode', Mock(return_value=direct_node))
    monkeypatch.setattr(hz, 'get_topic_names_and_types', Mock(
        return_value=[(topic, ['sensor_msgs/msg/Image']) for topic in topics]))
    monkeypatch.setattr(hz, 'get_msg_class', Mock(return_value=object))
    monkeypatch.setattr(hz, 'choose_qos', Mock())
    monkeypatch.setattr(hz.threading, 'Thread', Mock())
    monkeypatch.setattr(hz.rclpy, 'spin', Mock())
    monkeypatch.setattr(hz.rclpy, 'try_shutdown', Mock())

    assert hz.main(args) == 0
    assert node.create_subscription.call_count == len(topics)
    for topic, call in zip(topics, node.create_subscription.call_args_list):
        assert call.args[1] == topic
        assert call.kwargs['raw'] is (backends is None and not filtered)
        if backends is None:
            assert 'acceptable_buffer_backends' not in call.kwargs
        else:
            assert call.kwargs['acceptable_buffer_backends'] == backends

        # Timing and metadata filters must not inspect the payload.
        class Message:
            header = SimpleNamespace(frame_id='camera')

            @property
            def data(self):
                raise AssertionError('hz accessed the buffer payload')

        callback = call.args[2]
        if filtered:
            callback(SimpleNamespace(header=SimpleNamespace(frame_id='other')))
        callback(Message())
        callback(Message())
        stats = callback.func.__self__.get_hz(topic)
        assert stats[0] * 1e9 == pytest.approx(1.0)
        assert stats[4] == 1


def test_hz_buffer_backends_requires_value():
    parser = argparse.ArgumentParser()
    hz.HzVerb().add_arguments(parser, 'ros2 topic hz')
    with pytest.raises(SystemExit) as exc:
        parser.parse_args(['/image', '--acceptable-buffer-backends'])
    assert exc.value.code == 2
