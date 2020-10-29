# Copyright 2018 Open Source Robotics Foundation, Inc.
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
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import SetParameters
import rclpy
from ros2cli.node.direct import DirectNode
import yaml


def get_value(*, parameter_value):
    """Get the value from a ParameterValue."""
    if parameter_value.type == ParameterType.PARAMETER_BOOL:
        value = parameter_value.bool_value
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER:
        value = parameter_value.integer_value
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE:
        value = parameter_value.double_value
    elif parameter_value.type == ParameterType.PARAMETER_STRING:
        value = parameter_value.string_value
    elif parameter_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
        value = list(parameter_value.byte_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
        value = list(parameter_value.bool_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        value = list(parameter_value.integer_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        value = list(parameter_value.double_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_STRING_ARRAY:
        value = list(parameter_value.string_array_value)
    elif parameter_value.type == ParameterType.PARAMETER_NOT_SET:
        value = None
    else:
        value = None

    return value


def get_parameter_value(*, string_value):
    """Guess the desired type of the parameter based on the string value."""
    value = ParameterValue()
    try:
        yaml_value = yaml.safe_load(string_value)
    except yaml.parser.ParserError:
        value.type = ParameterType.PARAMETER_STRING
        value.string_value = string_value
        return value

    if isinstance(yaml_value, bool):
        value.type = ParameterType.PARAMETER_BOOL
        value.bool_value = yaml_value
    elif isinstance(yaml_value, int):
        value.type = ParameterType.PARAMETER_INTEGER
        value.integer_value = yaml_value
    elif isinstance(yaml_value, float):
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = yaml_value
    elif isinstance(yaml_value, list):
        if all((isinstance(v, bool) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_BOOL_ARRAY
            value.bool_array_value = yaml_value
        elif all((isinstance(v, int) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_INTEGER_ARRAY
            value.integer_array_value = yaml_value
        elif all((isinstance(v, float) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            value.double_array_value = yaml_value
        elif all((isinstance(v, str) for v in yaml_value)):
            value.type = ParameterType.PARAMETER_STRING_ARRAY
            value.string_array_value = yaml_value
        else:
            value.type = ParameterType.PARAMETER_STRING
            value.string_value = string_value
    else:
        value.type = ParameterType.PARAMETER_STRING
        value.string_value = string_value
    return value


def call_describe_parameters(*, node, node_name, parameter_names=None):
    # create client
    client = node.create_client(
        DescribeParameters, f'{node_name}/describe_parameters')

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = DescribeParameters.Request()
    if parameter_names:
        request.names = parameter_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            f"Exception while calling service of node '{node_name}': {e}")
    return response


def call_get_parameters(*, node, node_name, parameter_names):
    # create client
    client = node.create_client(GetParameters, f'{node_name}/get_parameters')

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = GetParameters.Request()
    request.names = parameter_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            f"Exception while calling service of node '{node_name}': {e}")
    return response


def call_set_parameters(*, node, node_name, parameters):
    # create client
    client = node.create_client(SetParameters, f'{node_name}/set_parameters')

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = SetParameters.Request()
    request.parameters = parameters
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            f"Exception while calling service of node '{node_name}': {e}")
    return response


def call_list_parameters(*, node, node_name, prefix=None):
    # create client
    client = node.create_client(ListParameters, f'{node_name}/list_parameters')

    # call as soon as ready
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = ListParameters.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            f"Exception while calling service of node '{node_name}': {e}")
    return response.result.names


def get_parameter_type_string(parameter_type):
    mapping = {
        ParameterType.PARAMETER_BOOL: 'boolean',
        ParameterType.PARAMETER_INTEGER: 'integer',
        ParameterType.PARAMETER_DOUBLE: 'double',
        ParameterType.PARAMETER_STRING: 'string',
        ParameterType.PARAMETER_BYTE_ARRAY: 'byte array',
        ParameterType.PARAMETER_BOOL_ARRAY: 'boolean array',
        ParameterType.PARAMETER_INTEGER_ARRAY: 'integer array',
        ParameterType.PARAMETER_DOUBLE_ARRAY: 'double array',
        ParameterType.PARAMETER_STRING_ARRAY: 'string array',
        ParameterType.PARAMETER_NOT_SET: 'not set',
    }
    return mapping[parameter_type]


class ParameterNameCompleter:
    """Callable returning a list of parameter names."""

    def __call__(self, prefix, parsed_args, **kwargs):
        with DirectNode(parsed_args) as node:
            parameter_names = call_list_parameters(
                node=node, node_name=parsed_args.node_name)
            return [
                n for n in parameter_names
                if not prefix or n.startswith(prefix)]
