# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.node import Node


class ParamNode(Node):

    def __init__(self):
        super().__init__('param_node')
        self.declare_parameter('bool_param', True, ParameterDescriptor())
        self.declare_parameter('int_param', 42, ParameterDescriptor())
        self.declare_parameter('double_param', 1.23, ParameterDescriptor())
        self.declare_parameter('str_param', 'Hello World', ParameterDescriptor())
        self.declare_parameter('int_array', [1, 2, 3], ParameterDescriptor())
        self.declare_parameter('double_array', [1.0, 2.0, 3.0], ParameterDescriptor())
        self.declare_parameter('bool_array', [True, False, True], ParameterDescriptor())
        self.declare_parameter('str_array', ['Hello', 'World'], ParameterDescriptor())
        self.declare_parameter('byte_array', [b'p', b'v'], ParameterDescriptor())
        self.declare_parameter('parameter_with_no_value', None, ParameterDescriptor())


def main(args=None):
    rclpy.init(args=args)

    node = ParamNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('node stopped cleanly')
    except BaseException:
        print('exception in node:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
