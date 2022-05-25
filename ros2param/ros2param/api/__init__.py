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

import rclpy
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter import get_parameter_value as rclpy_get_parameter_value
from rclpy.parameter_client import AsyncParameterClient

from ros2cli.node.direct import DirectNode


def get_parameter_value(*, string_value):
    rclpy_get_parameter_value(string_value)


def get_value(*, parameter_value):
    """Get the value from a ParameterValue."""
    value = None
    try:
        value = parameter_value_to_python(parameter_value)
    except RuntimeError as e:
        print(e)
    # NOTE(ihasdapie): parameter_value_to_python raises an error if the type is not supported but 
    # get_value would just return value=None
    return value


def load_parameter_file(*, node, node_name, parameter_file, use_wildcard):
    # Remove leading slash and namespaces
    client = AsyncParameterClient(node, node_name)
    (future, parameters) = client.load_parameter_file(parameter_file, use_wildcard, return_parameters = True)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    assert len(response.results) == len(parameters), "Not all parameters set"
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


def call_describe_parameters(*, node, node_name, parameter_names=None):
    client = AsyncParameterClient(node, node_name)
    future = client.describe_parameters(parameter_names)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    return response


def call_get_parameters(*, node, node_name, parameter_names):
    """
    :param node: node to create client on
    :param node_name: name of node to set parameters on
    """
    client = AsyncParameterClient(node, node_name)
    future = client.get_parameters(parameter_names)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    return response


def call_set_parameters(*, node, node_name, parameters):
    """
    :param node: node to create client on
    :param node_name: name of node to set parameters on
    """
    client = AsyncParameterClient(node, node_name)
    future = client.set_parameters(parameters)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    return response


def call_list_parameters(*, node, node_name, prefix=None):
    """
    :param node: node to create client on
    :param node_name: name of node to set parameters on
    """
    client = AsyncParameterClient(node, node_name)
    future = client.list_parameters()
    rclpy.spin_until_future_complete(node, future)
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
