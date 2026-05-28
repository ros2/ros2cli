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


def test_topic_name_completer():
    from unittest.mock import Mock, patch
    from ros2topic.api import TopicNameCompleter

    parsed_args = Mock()
    parsed_args.include_hidden_topics = False
    completer = TopicNameCompleter(include_hidden_topics_key='include_hidden_topics')

    # Test 1: returns topic names
    with patch('ros2topic.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2topic.api.get_topic_names') as mock_get_names:
            mock_get_names.return_value = ['/topic1', '/topic2']
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == ['/topic1', '/topic2']
            mock_get_names.assert_called_once_with(
                node=mock_node, include_hidden_topics=False)

    # Test 2: no topics available
    with patch('ros2topic.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2topic.api.get_topic_names') as mock_get_names:
            mock_get_names.return_value = []
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == []

    # Test 3: include hidden topics
    parsed_args.include_hidden_topics = True
    with patch('ros2topic.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2topic.api.get_topic_names') as mock_get_names:
            mock_get_names.return_value = ['/topic1', '/_hidden_topic']
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == ['/topic1', '/_hidden_topic']
            mock_get_names.assert_called_once_with(
                node=mock_node, include_hidden_topics=True)


def test_topic_type_completer():
    from unittest.mock import Mock, patch
    from ros2topic.api import TopicTypeCompleter

    parsed_args = Mock()

    # Test 1: no topic_name_key, returns all message types
    completer = TopicTypeCompleter()
    with patch('ros2topic.api.message_type_completer') as mock_all_types:
        mock_all_types.return_value = ['std_msgs/String', 'std_msgs/Int32']
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == ['std_msgs/String', 'std_msgs/Int32']
        mock_all_types.assert_called_once()

    # Test 2: topic_name_key provided and topic found
    parsed_args.topic_name = '/my_topic'
    completer = TopicTypeCompleter(topic_name_key='topic_name')
    with patch('ros2topic.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2topic.api.get_topic_names_and_types') as mock_get_names_types:
            mock_get_names_types.return_value = [
                ('/my_topic', ['std_msgs/String']),
                ('/other_topic', ['std_msgs/Int32']),
            ]
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == ['std_msgs/String']

    # Test 3: topic_name_key provided but topic not found, falls back to all types
    parsed_args.topic_name = '/nonexistent_topic'
    completer = TopicTypeCompleter(topic_name_key='topic_name')
    with patch('ros2topic.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2topic.api.get_topic_names_and_types') as mock_get_names_types:
            mock_get_names_types.return_value = [
                ('/my_topic', ['std_msgs/String']),
            ]
            with patch('ros2topic.api.message_type_completer') as mock_all_types:
                mock_all_types.return_value = ['std_msgs/String', 'std_msgs/Int32']
                result = completer(prefix='', parsed_args=parsed_args)
                assert result == ['std_msgs/String', 'std_msgs/Int32']


def test_topic_message_prototype_completer():
    from unittest.mock import Mock, patch
    from ros2topic.api import TopicMessagePrototypeCompleter

    parsed_args = Mock()
    parsed_args.message_type = 'std_msgs/String'
    completer = TopicMessagePrototypeCompleter(topic_type_key='message_type')

    mock_message_class = Mock()
    mock_message_instance = Mock()
    mock_message_class.return_value = mock_message_instance

    with patch('ros2topic.api.get_message') as mock_get_message:
        with patch('ros2topic.api.message_to_yaml') as mock_message_to_yaml:
            mock_get_message.return_value = mock_message_class
            mock_message_to_yaml.return_value = 'data: ""'

            result = completer(prefix='', parsed_args=parsed_args)

            mock_get_message.assert_called_once_with('std_msgs/String')
            mock_message_to_yaml.assert_called_once_with(mock_message_instance)
            assert result == ["'data: \"\"'"]
