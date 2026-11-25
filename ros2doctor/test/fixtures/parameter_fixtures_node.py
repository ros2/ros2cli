# Copyright 2025 Open Source Robotics Foundation, Inc.
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

"""Test fixture node with parameters for ros2doctor testing."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import PARAMETER_SEPARATOR_STRING


def main():
    try:
        with rclpy.init():
            node = rclpy.create_node('parameter_node')
            # Declare various parameter types for testing
            node.declare_parameter('bool_param', True)
            node.declare_parameter('int_param', 42)
            node.declare_parameter('double_param', 3.14)
            node.declare_parameter('str_param', 'hello')
            node.declare_parameter('bool_array_param', [True, False])
            node.declare_parameter('int_array_param', [1, 2, 3])
            node.declare_parameter('double_array_param', [1.0, 2.0])
            node.declare_parameter('str_array_param', ['foo', 'bar'])
            node.declare_parameter(
                'nested' + PARAMETER_SEPARATOR_STRING + 'param', 'nested_value'
            )
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('parameter_node stopped cleanly')


if __name__ == '__main__':
    main()
