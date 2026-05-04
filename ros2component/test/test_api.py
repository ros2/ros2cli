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

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy

from ros2component.api import find_container_node_names
from ros2component.api import get_package_component_types

from ros2node.api import get_node_names


def test_find_container_node_names():
    """Test find_container_node_names() API function."""
    with NodeStrategy([]) as node:
        node_names = get_node_names(node=node)

    with DirectNode([]) as node:
        assert len(find_container_node_names(
            node=node, node_names=node_names
        )) == 0

        assert len(find_container_node_names(
            node=node, node_names=[]
        )) == 0


def test_get_package_component_types():
    """Test get_package_component_types() API function."""
    assert len(get_package_component_types(package_name='ros2component')) == 0


def test_component_type_name_completer():
    from unittest.mock import Mock, patch
    from ros2component.api import ComponentTypeNameCompleter

    parsed_args = Mock()
    parsed_args.package_name = 'my_package'
    completer = ComponentTypeNameCompleter(package_name_key='package_name')

    # Test 1: package has component types
    with patch('ros2component.api.get_package_component_types') as mock_get_types:
        mock_get_types.return_value = ['my_package::MyComponent', 'my_package::OtherComponent']
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == ['my_package::MyComponent', 'my_package::OtherComponent']
        mock_get_types.assert_called_once_with(package_name='my_package')

    # Test 2: package has no component types
    with patch('ros2component.api.get_package_component_types') as mock_get_types:
        mock_get_types.return_value = []
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == []

    # Test 3: different package name
    parsed_args.package_name = 'other_package'
    with patch('ros2component.api.get_package_component_types') as mock_get_types:
        mock_get_types.return_value = ['other_package::Comp']
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == ['other_package::Comp']
        mock_get_types.assert_called_once_with(package_name='other_package')
