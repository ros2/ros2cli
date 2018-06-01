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
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
import rclpy


def get_parameter_value(*, string_value):
    """Guess the desired type of the parameter based on the string value."""
    value = ParameterValue()
    if string_value.lower() in ('true', 'false'):
        value.type = ParameterType.PARAMETER_BOOL
        value.bool_value = string_value.lower() == 'true'
    elif _is_integer(string_value):
        value.type = ParameterType.PARAMETER_INTEGER
        value.integer_value = int(string_value)
    elif _is_float(string_value):
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = float(string_value)
    else:
        value.type = ParameterType.PARAMETER_STRING
        value.string_value = string_value
    return value


def _is_integer(string_value):
    try:
        integer_value = int(string_value)
    except ValueError:
        return False
    return str(integer_value) == string_value


def _is_float(string_value):
    try:
        float(string_value)
    except ValueError:
        return False
    return True


def call_get_parameters(*, node, node_name, parameter_names):
    # create client
    client = node.create_client(
        GetParameters,
        '/{node_name}/get_parameters'.format_map(locals()))

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
            'Exception while calling service of node '
            "'{args.node_name}': {e}".format_map(locals()))
    return response


def call_set_parameters(*, node, node_name, parameters):
    # create client
    client = node.create_client(
        SetParameters,
        '/{node_name}/set_parameters'.format_map(locals()))

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
            'Exception while calling service of node '
            "'{args.node_name}': {e}".format_map(locals()))
    return response
