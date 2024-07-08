# Copyright 2020 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import PARAMETER_SEPARATOR_STRING


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = rclpy.create_node('parameter_node')
            node.declare_parameter('bool_param', True)
            node.declare_parameter('int_param', 42)
            node.declare_parameter('double_param', 1.23)
            node.declare_parameter('str_param', 'Hello World')
            node.declare_parameter('bool_array_param', [False, False, True])
            node.declare_parameter('int_array_param', [1, 2, 3])
            node.declare_parameter('str_array_param', ['foo', 'bar', 'baz'])
            node.declare_parameter('double_array_param', [3.125, 6.25, 12.5])
            node.declare_parameter('foo' + PARAMETER_SEPARATOR_STRING + 'str_param', 'foo')
            node.declare_parameter('foo' + PARAMETER_SEPARATOR_STRING +
                                   'bar' + PARAMETER_SEPARATOR_STRING +
                                   'str_param', 'foobar')

            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('parameter node stopped cleanly')


if __name__ == '__main__':
    main()
