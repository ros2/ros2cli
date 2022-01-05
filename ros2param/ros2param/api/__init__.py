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

import sys

from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import PARAMETER_SEPARATOR_STRING
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
        yaml_value = string_value

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
        value.string_value = yaml_value
    return value


def parse_parameter_dict(*, namespace, parameter_dict):
    parameters = []
    for param_name, param_value in parameter_dict.items():
        full_param_name = namespace + param_name
        # Unroll nested parameters
        if type(param_value) == dict:
            parameters += parse_parameter_dict(
                    namespace=full_param_name + PARAMETER_SEPARATOR_STRING,
                    parameter_dict=param_value)
        else:
            parameter = Parameter()
            parameter.name = full_param_name
            parameter.value = get_parameter_value(string_value=str(param_value))
            parameters.append(parameter)
    return parameters


def load_parameter_dict(*, node, node_name, parameter_dict):

    parameters = parse_parameter_dict(namespace='', parameter_dict=parameter_dict)
    response = call_set_parameters(
        node=node, node_name=node_name, parameters=parameters)

    # output response
    assert len(response.results) == len(parameters)
    for i in range(0, len(response.results)):
        result = response.results[i]
        param_name = parameters[i].name
        if result.successful:
            msg = 'Set parameter {} successful'.format(param_name)
            if result.reason:
                msg += ': ' + result.reason
            print(msg)
        else:
            msg = 'Set parameter {} failed'.format(param_name)
            if result.reason:
                msg += ': ' + result.reason
            print(msg, file=sys.stderr)


def load_parameter_file(*, node, node_name, parameter_file, use_wildcard):
    # Remove leading slash and namespaces
    with open(parameter_file, 'r') as f:
        param_file = yaml.safe_load(f)
        param_keys = []
        if use_wildcard and '/**' in param_file:
            param_keys.append('/**')
        if node_name in param_file:
            param_keys.append(node_name)

        if param_keys == []:
            raise RuntimeError('Param file does not contain parameters for {}, '
                               ' only for nodes: {}' .format(node_name, param_file.keys()))
        param_dict = {}
        for k in param_keys:
            value = param_file[k]
            if type(value) != dict or 'ros__parameters' not in value:
                raise RuntimeError('Invalid structure of parameter file for node {}'
                                   'expected same format as provided by ros2 param dump'
                                   .format(k))
            param_dict.update(value['ros__parameters'])
        load_parameter_dict(node=node, node_name=node_name, parameter_dict=param_dict)


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
