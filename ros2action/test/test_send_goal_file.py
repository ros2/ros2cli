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

from ros2action.verb.send_goal import SendGoalVerb


def test_send_goal_reads_goal_file(tmp_path):
    goal_file = tmp_path / 'goal.yaml'
    goal_file.write_text('order: 7\n', encoding='utf-8')

    parser = ArgumentParser()
    verb = SendGoalVerb()
    verb.add_arguments(parser, 'action')
    args = parser.parse_args([
        '/fibonacci',
        'example_interfaces/action/Fibonacci',
        '--goal-file', str(goal_file),
    ])

    with patch('ros2action.verb.send_goal.send_goal') as mock_send_goal:
        verb.main(args=args)

    mock_send_goal.assert_called_once_with(
        '/fibonacci',
        'example_interfaces/action/Fibonacci',
        'order: 7\n',
        None,
        None,
    )
