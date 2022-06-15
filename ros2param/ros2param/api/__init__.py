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
import time
import warnings

from rcl_interfaces.msg import ParameterType
import rclpy
from rclpy.parameter import get_parameter_value as rclpy_get_parameter_value
from rclpy.parameter import parameter_dict_from_yaml_file
from rclpy.parameter import parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from ros2cli.node.direct import DirectNode


def get_parameter_value(*, string_value):
    warnings.warn('get_parameter_value() is deprecated. '
                  'Use rclpy.parameter.get_parameter_value instead')
    return rclpy_get_parameter_value(string_value)


def get_value(*, parameter_value):
    """Get the value from a ParameterValue."""
    value = None
    try:
        value = parameter_value_to_python(parameter_value)
    except RuntimeError as e:
        print(f'Runtime error {e} raised')
    return value


def busy_wait_client(client, node):
    timeout_sec = 5
    prev = time.time()
    while not (client.services_are_ready() or timeout_sec < 0):
        client.wait_for_services(timeout_sec=0.1)
        rclpy.spin_once(node)
        timeout_sec -= prev - time.time()
    if not client.services_are_ready():
        raise RuntimeError('Could not reach parameter services')


def load_parameter_file(*, node, node_name, parameter_file, use_wildcard):
    client = AsyncParameterClient(node, node_name)
    busy_wait_client(client, node)
    future = client.load_parameter_file(parameter_file, use_wildcard)
    parameters = list(parameter_dict_from_yaml_file(parameter_file, use_wildcard).values())
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    assert len(response.results) == len(parameters), 'Not all parameters set'
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
    busy_wait_client(client, node)

    future = client.describe_parameters(parameter_names)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if response is None:
        raise RuntimeError('Exception while calling service of node '
                           f'{node_name}: {future.exception()}')
    return response


def call_get_parameters(*, node, node_name, parameter_names):
    client = AsyncParameterClient(node, node_name)
    busy_wait_client(client, node)

    future = client.get_parameters(parameter_names)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if response is None:
        raise RuntimeError('Exception while calling service of node '
                           f'{node_name}: {future.exception()}')
    return response


def call_set_parameters(*, node, node_name, parameters):
    client = AsyncParameterClient(node, node_name)
    busy_wait_client(client, node)

    future = client.set_parameters(parameters)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if response is None:
        raise RuntimeError('Exception while calling service of node '
                           f'{node_name}: {future.exception()}')
    return response


def call_list_parameters(*, node, node_name, prefixes=None):
    client = AsyncParameterClient(node, node_name)
    busy_wait_client(client, node)

    future = client.list_parameters(prefixes=prefixes)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    if response is None:
        raise RuntimeError('Exception while calling service of node '
                           f'{node_name}: {future.exception()}')
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
