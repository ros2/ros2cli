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

from rcl_interfaces.msg import ParameterType
from ros2param.api import get_parameter_value


def test_bool():
    value = get_parameter_value(string_value='true')
    assert value.type == ParameterType.PARAMETER_BOOL
    assert value.bool_value is True

    value = get_parameter_value(string_value='false')
    assert value.type == ParameterType.PARAMETER_BOOL
    assert value.bool_value is False


def test_integer():
    value = get_parameter_value(string_value='1')
    assert value.type == ParameterType.PARAMETER_INTEGER
    assert value.integer_value == 1

    value = get_parameter_value(string_value='0')
    assert value.type == ParameterType.PARAMETER_INTEGER
    assert value.integer_value == 0

    value = get_parameter_value(string_value='-1')
    assert value.type == ParameterType.PARAMETER_INTEGER
    assert value.integer_value == -1


def test_double():
    value = get_parameter_value(string_value='1.0')
    assert value.type == ParameterType.PARAMETER_DOUBLE
    assert value.double_value == 1

    value = get_parameter_value(string_value='0.0')
    assert value.type == ParameterType.PARAMETER_DOUBLE
    assert value.double_value == 0

    value = get_parameter_value(string_value='-1.0')
    assert value.type == ParameterType.PARAMETER_DOUBLE
    assert value.double_value == -1

    value = get_parameter_value(string_value='1.1234')
    assert value.type == ParameterType.PARAMETER_DOUBLE
    assert value.double_value == 1.1234


def test_string():
    value = get_parameter_value(string_value="foo")
    assert value.type == ParameterType.PARAMETER_STRING
    assert value.string_value == "foo"


def test_bool_array():
    value = get_parameter_value(string_value='[false, true]')
    assert value.type == ParameterType.PARAMETER_BOOL_ARRAY
    assert value.bool_array_value == [False, True]


def test_integer_array():
    value = get_parameter_value(string_value='[-1, 0, 1]')
    assert value.type == ParameterType.PARAMETER_INTEGER_ARRAY
    assert value.integer_array_value == [-1, 0, 1]


def test_float_array():
    value = get_parameter_value(string_value='[-1.0, 0.0, 1.0, 1.234]')
    assert value.type == ParameterType.PARAMETER_DOUBLE_ARRAY
    assert value.double_array_value == [-1.0, 0.0, 1.0, 1.234]


def test_string_array():
    value = get_parameter_value(string_value='["foo", "bar", "buzz"]')
    assert value.type == ParameterType.PARAMETER_STRING_ARRAY
    assert value.string_array_value == ["foo", "bar", "buzz"]


def test_malformed_yaml():
    value = get_parameter_value(string_value='["foo","bar","buzz"')
    assert value.type == ParameterType.PARAMETER_STRING
    assert value.string_value == '["foo","bar","buzz"'
