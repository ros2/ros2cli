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


def test_service_name_completer():
    from unittest.mock import Mock, patch
    from ros2service.api import ServiceNameCompleter

    parsed_args = Mock()
    parsed_args.include_hidden_services = False
    completer = ServiceNameCompleter(
        include_hidden_services_key='include_hidden_services')

    # Test 1: returns service names
    with patch('ros2service.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2service.api.get_service_names') as mock_get_names:
            mock_get_names.return_value = ['/service1', '/service2']
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == ['/service1', '/service2']
            mock_get_names.assert_called_once_with(
                node=mock_node, include_hidden_services=False)

    # Test 2: no services available
    with patch('ros2service.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2service.api.get_service_names') as mock_get_names:
            mock_get_names.return_value = []
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == []

    # Test 3: include hidden services
    parsed_args.include_hidden_services = True
    with patch('ros2service.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2service.api.get_service_names') as mock_get_names:
            mock_get_names.return_value = ['/service1', '/_hidden_service']
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == ['/service1', '/_hidden_service']
            mock_get_names.assert_called_once_with(
                node=mock_node, include_hidden_services=True)


def test_service_type_completer():
    from unittest.mock import Mock, patch
    from ros2service.api import ServiceTypeCompleter

    parsed_args = Mock()

    # Test 1: no service_name_key, returns all service types
    completer = ServiceTypeCompleter()
    with patch('ros2service.api.service_type_completer') as mock_all_types:
        mock_all_types.return_value = ['std_srvs/Empty', 'std_srvs/SetBool']
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == ['std_srvs/Empty', 'std_srvs/SetBool']
        mock_all_types.assert_called_once()

    # Test 2: service_name_key provided and service found
    parsed_args.service_name = '/my_service'
    completer = ServiceTypeCompleter(service_name_key='service_name')
    with patch('ros2service.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2service.api.get_service_names_and_types') as mock_get_names_types:
            mock_get_names_types.return_value = [
                ('/my_service', ['std_srvs/Empty']),
                ('/other_service', ['std_srvs/SetBool']),
            ]
            result = completer(prefix='', parsed_args=parsed_args)
            assert result == ['std_srvs/Empty']

    # Test 3: service_name_key provided but service not found, falls back to all types
    parsed_args.service_name = '/nonexistent_service'
    completer = ServiceTypeCompleter(service_name_key='service_name')
    with patch('ros2service.api.NodeStrategy') as mock_node_strategy:
        mock_node = Mock()
        mock_node_strategy.return_value.__enter__ = Mock(return_value=mock_node)
        mock_node_strategy.return_value.__exit__ = Mock(return_value=False)
        with patch('ros2service.api.get_service_names_and_types') as mock_get_names_types:
            mock_get_names_types.return_value = [
                ('/my_service', ['std_srvs/Empty']),
            ]
            with patch('ros2service.api.service_type_completer') as mock_all_types:
                mock_all_types.return_value = ['std_srvs/Empty', 'std_srvs/SetBool']
                result = completer(prefix='', parsed_args=parsed_args)
                assert result == ['std_srvs/Empty', 'std_srvs/SetBool']


def test_service_prototype_completer():
    from unittest.mock import Mock, patch
    from ros2service.api import ServicePrototypeCompleter

    parsed_args = Mock()
    parsed_args.service_type = 'std_srvs/SetBool'
    completer = ServicePrototypeCompleter(service_type_key='service_type')

    mock_service_class = Mock()
    mock_request_instance = Mock()
    mock_service_class.Request.return_value = mock_request_instance

    with patch('ros2service.api.get_service') as mock_get_service:
        with patch('ros2service.api.message_to_yaml') as mock_message_to_yaml:
            mock_get_service.return_value = mock_service_class
            mock_message_to_yaml.return_value = 'data: false'

            result = completer(prefix='', parsed_args=parsed_args)

            mock_get_service.assert_called_once_with('std_srvs/SetBool')
            mock_message_to_yaml.assert_called_once_with(mock_request_instance)
            assert result == ['data: false']
