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


def test_node_name_completer():
    from unittest.mock import Mock, patch
    from ros2node.api import NodeNameCompleter, NodeName

    parsed_args = Mock()

    # Test 1: no include_hidden_nodes_key, defaults to False
    completer = NodeNameCompleter()
    with patch('ros2node.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2node.api.get_node_names') as mock_get_names:
            mock_get_names.return_value = [
                NodeName('node1', '/', '/node1'),
                NodeName('node2', '/ns', '/ns/node2'),
            ]
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == ['/node1', '/ns/node2']
            mock_get_names.assert_called_once_with(
                node=mock_node, include_hidden_nodes=False)

    # Test 2: include_hidden_nodes_key provided and True
    parsed_args.include_hidden_nodes = True
    completer = NodeNameCompleter(include_hidden_nodes_key='include_hidden_nodes')
    with patch('ros2node.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2node.api.get_node_names') as mock_get_names:
            mock_get_names.return_value = [
                NodeName('node1', '/', '/node1'),
                NodeName('_hidden', '/', '/_hidden'),
            ]
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == ['/node1', '/_hidden']
            mock_get_names.assert_called_once_with(
                node=mock_node, include_hidden_nodes=True)

    # Test 3: no nodes available
    parsed_args.include_hidden_nodes = False
    completer = NodeNameCompleter(include_hidden_nodes_key='include_hidden_nodes')
    with patch('ros2node.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2node.api.get_node_names') as mock_get_names:
            mock_get_names.return_value = []
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == []
