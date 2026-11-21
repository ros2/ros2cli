# Copyright 2026 Open Source Robotics Foundation, Inc.
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

from argparse import ArgumentParser
from unittest.mock import patch

import pytest

from ros2action.verb.send_goal import SendGoalVerb


def _parse_goal_file(path):
    parser = ArgumentParser()
    verb = SendGoalVerb()
    verb.add_arguments(parser, 'action')
    args = parser.parse_args([
        '/fibonacci',
        'example_interfaces/action/Fibonacci',
        '--goal-file', str(path),
    ])
    return verb, args


def test_send_goal_reads_goal_file_when_command_runs(tmp_path):
    goal_file = tmp_path / 'goal.yaml'
    goal_file.write_text('order: 7\n', encoding='utf-8')
    verb, args = _parse_goal_file(goal_file)

    assert args.goal_file == str(goal_file)

    with patch('ros2action.verb.send_goal.send_goal') as mock_send_goal:
        result = verb.main(args=args)

    assert result is mock_send_goal.return_value
    mock_send_goal.assert_called_once_with(
        '/fibonacci',
        'example_interfaces/action/Fibonacci',
        'order: 7\n',
        None,
        None,
    )


def test_send_goal_rejects_empty_goal_file(tmp_path):
    goal_file = tmp_path / 'empty.yaml'
    goal_file.write_text('', encoding='utf-8')
    verb, args = _parse_goal_file(goal_file)

    with patch('ros2action.verb.send_goal.send_goal') as mock_send_goal:
        result = verb.main(args=args)

    assert result == 'Goal file is empty'
    mock_send_goal.assert_not_called()


def test_send_goal_rejects_empty_goal_file_path():
    verb, args = _parse_goal_file('')

    with patch('ros2action.verb.send_goal.send_goal') as mock_send_goal:
        result = verb.main(args=args)

    assert result.startswith('Failed to read goal file:')
    mock_send_goal.assert_not_called()


def test_send_goal_reports_non_utf8_goal_file(tmp_path):
    goal_file = tmp_path / 'bad-encoding.yaml'
    goal_file.write_bytes(b'order: \xff\n')
    verb, args = _parse_goal_file(goal_file)

    with patch('ros2action.verb.send_goal.send_goal') as mock_send_goal:
        result = verb.main(args=args)

    assert result.startswith('Failed to read goal file:')
    mock_send_goal.assert_not_called()


def test_send_goal_reports_malformed_goal_file(tmp_path):
    goal_file = tmp_path / 'bad-yaml.yaml'
    goal_file.write_text('order: [1, 2\n', encoding='utf-8')
    verb, args = _parse_goal_file(goal_file)

    with patch('ros2action.verb.send_goal.send_goal') as mock_send_goal:
        result = verb.main(args=args)

    assert result.startswith('Failed to parse goal file:')
    mock_send_goal.assert_not_called()


@pytest.mark.parametrize('text', ['# TODO\n', '---\n', 'null\n'])
def test_send_goal_rejects_yaml_without_goal_data(tmp_path, text):
    goal_file = tmp_path / 'empty-goal.yaml'
    goal_file.write_text(text, encoding='utf-8')
    verb, args = _parse_goal_file(goal_file)

    with patch('ros2action.verb.send_goal.send_goal') as mock_send_goal:
        result = verb.main(args=args)

    assert result == 'Goal file is empty'
    mock_send_goal.assert_not_called()


@pytest.mark.parametrize('text', ['[]\n', 'hello\n', '42\n'])
def test_send_goal_rejects_non_mapping_goal_file(tmp_path, text):
    goal_file = tmp_path / 'not-mapping.yaml'
    goal_file.write_text(text, encoding='utf-8')
    verb, args = _parse_goal_file(goal_file)

    with patch('ros2action.verb.send_goal.send_goal') as mock_send_goal:
        result = verb.main(args=args)

    assert result == 'Goal file must contain a YAML mapping'
    mock_send_goal.assert_not_called()


def test_send_goal_reports_goal_file_read_error(tmp_path):
    missing_goal_file = tmp_path / 'missing.yaml'
    verb, args = _parse_goal_file(missing_goal_file)

    with patch('ros2action.verb.send_goal.send_goal') as mock_send_goal:
        result = verb.main(args=args)

    assert result.startswith('Failed to read goal file:')
    assert str(missing_goal_file) in result
    mock_send_goal.assert_not_called()
