# Copyright 2017 Open Source Robotics Foundation, Inc.
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

import array

import pytest

from rcl_interfaces.msg import ParameterType
from ros2param.api import get_parameter_value


@pytest.mark.parametrize(
    'string_value,expected_type,value_attribute,expected_value',
    [
        ('true', ParameterType.PARAMETER_BOOL, 'bool_value', True),
        ('false', ParameterType.PARAMETER_BOOL, 'bool_value', False),

        ('1', ParameterType.PARAMETER_INTEGER, 'integer_value', 1),
        ('0', ParameterType.PARAMETER_INTEGER, 'integer_value', 0),
        ('-1', ParameterType.PARAMETER_INTEGER, 'integer_value', -1),

        ('1.0', ParameterType.PARAMETER_DOUBLE, 'double_value', 1.0),
        ('0.0', ParameterType.PARAMETER_DOUBLE, 'double_value', 0.0),
        ('-1.0', ParameterType.PARAMETER_DOUBLE, 'double_value', -1.0),
        ('1.1234', ParameterType.PARAMETER_DOUBLE, 'double_value', 1.1234),

        ('foo', ParameterType.PARAMETER_STRING, 'string_value', 'foo'),
        (
            '[false, true]',
            ParameterType.PARAMETER_BOOL_ARRAY,
            'bool_array_value',
            [False, True],
        ),
        (
            '[-1, 0, 1]',
            ParameterType.PARAMETER_INTEGER_ARRAY,
            'integer_array_value',
            array.array('q', (-1, 0, 1))
        ),
        (
            '[-1.0, 0.0, 1.0, 1.1234]',
            ParameterType.PARAMETER_DOUBLE_ARRAY,
            'double_array_value',
            array.array('d', (-1.0, 0.0, 1.0, 1.1234)),
        ),
        (
            '["foo", "bar", "buzz"]',
            ParameterType.PARAMETER_STRING_ARRAY,
            'string_array_value',
            ['foo', 'bar', 'buzz'],
        ),
        (  # Invalid YAML remains a string
            '["foo", "bar", "buzz"',
            ParameterType.PARAMETER_STRING,
            'string_value',
            '["foo", "bar", "buzz"',
        ),
        (
            # With 'off', YAML interprets this as a bool
            'off',
            ParameterType.PARAMETER_BOOL,
            'bool_value',
            False
        ),
        (
            # With !!str, text that would otherwise be a bool is a string
            '!!str off',
            ParameterType.PARAMETER_STRING,
            'string_value',
            'off'
        ),
        (
            # While YAML supports a mixed-type list, ROS 2 parameters do not,
            # so these end up being strings
            '[true,0.1,1]',
            ParameterType.PARAMETER_STRING,
            'string_value',
            '[true,0.1,1]'
        ),
    ],
)


# TODO(ihasapie): Move into rclpy
# def test_get_parameter_value(
#     string_value, expected_type, value_attribute, expected_value
# ):
#     value = get_parameter_value(string_value=string_value)
#     assert value.type == expected_type
#     assert getattr(value, value_attribute) == expected_value
