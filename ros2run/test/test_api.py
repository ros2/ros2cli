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


def test_executable_name_completer():
    from unittest.mock import Mock, patch
    from ros2pkg.api import PackageNotFound
    from ros2run.api import ExecutableNameCompleter

    parsed_args = Mock()
    parsed_args.package_name = 'my_package'
    completer = ExecutableNameCompleter(package_name_key='package_name')

    # Test 1: package has executables
    with patch('ros2run.api.get_executable_paths') as mock_get_paths:
        mock_get_paths.return_value = [
            '/opt/ros/humble/lib/my_package/my_exec',
            '/opt/ros/humble/lib/my_package/other_exec',
        ]
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == ['my_exec', 'other_exec']
        mock_get_paths.assert_called_once_with(package_name='my_package')

    # Test 2: package not found raises PackageNotFound, returns empty list
    with patch('ros2run.api.get_executable_paths') as mock_get_paths:
        mock_get_paths.side_effect = PackageNotFound('my_package')
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == []

    # Test 3: package has no executables
    with patch('ros2run.api.get_executable_paths') as mock_get_paths:
        mock_get_paths.return_value = []
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == []
