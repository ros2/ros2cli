# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from ros2action.api import get_action_clients_and_servers
from ros2action.api import get_action_names
from ros2cli.node.strategy import DirectNode


def test_get_action_clients_and_servers():
    with DirectNode(None) as node:
        clients, servers = get_action_clients_and_servers(
            node=node,
            action_name='/test_action_name',
        )
    assert len(clients) == 0
    assert len(servers) == 0


def test_get_action_names():
    with DirectNode(None) as node:
        get_action_names(node=node)


def test_action_type_completer():
    from unittest.mock import Mock, patch
    from ros2action.api import ActionTypeCompleter

    # Create a mock parsed_args object
    parsed_args = Mock()

    # Test 1: When action_name_key is None, should return all action types
    completer = ActionTypeCompleter()
    with patch('ros2action.api.action_type_completer') as mock_action_type_completer:
        mock_action_type_completer.return_value = [
            'example_interfaces/Fibonacci',
            'action_tutorials_interfaces/Fibonacci'
        ]
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == ['example_interfaces/Fibonacci', 'action_tutorials_interfaces/Fibonacci']
        mock_action_type_completer.assert_called_once()

    # Test 2: When action_name_key is provided and action exists
    parsed_args.action_name = '/test_action'
    completer = ActionTypeCompleter(action_name_key='action_name')

    with patch('ros2action.api.NodeStrategy'):
        with patch('ros2action.api.get_action_names_and_types') as mock_get_names_types:
            mock_get_names_types.return_value = [
                ('/test_action', ['example_interfaces/Fibonacci']),
                ('/other_action', ['action_tutorials_interfaces/Fibonacci'])
            ]
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == ['example_interfaces/Fibonacci']

    # Test 3: When action_name_key is provided but action not found
    parsed_args.action_name = '/nonexistent_action'
    completer = ActionTypeCompleter(action_name_key='action_name')

    with patch('ros2action.api.NodeStrategy'):
        with patch('ros2action.api.get_action_names_and_types') as mock_get_names_types:
            mock_get_names_types.return_value = [
                ('/test_action', ['example_interfaces/Fibonacci']),
                ('/other_action', ['action_tutorials_interfaces/Fibonacci'])
            ]
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == []


def test_action_goal_prototype_completer():
    from unittest.mock import Mock, patch
    from ros2action.api import ActionGoalPrototypeCompleter

    # Create a mock parsed_args object
    parsed_args = Mock()
    parsed_args.action_type = 'example_interfaces/Fibonacci'

    completer = ActionGoalPrototypeCompleter(action_type_key='action_type')

    # Mock the action class and its Goal
    mock_action_class = Mock()
    mock_goal_instance = Mock()
    mock_action_class.Goal.return_value = mock_goal_instance

    with patch('ros2action.api.get_action') as mock_get_action:
        with patch('ros2action.api.message_to_yaml') as mock_message_to_yaml:
            mock_get_action.return_value = mock_action_class
            mock_message_to_yaml.return_value = 'order: 5'

            result = completer(prefix='', parsed_args=parsed_args)

            # Verify get_action was called with the correct action type
            mock_get_action.assert_called_once_with('example_interfaces/Fibonacci')
            # Verify message_to_yaml was called with the goal instance
            mock_message_to_yaml.assert_called_once_with(mock_goal_instance)
            # Verify the result is a list containing the YAML string
            assert result == ['order: 5']
