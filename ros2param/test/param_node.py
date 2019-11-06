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

    @staticmethod
    def get_node_parameters():
        return {
            'bool_param': True,
            'int_param': 42,
            'double_param': 1.23,
            'str_param': 'Hello World',
            'int_array': [1, 2, 3],
            'double_array': [1.0, 2.0, 3.0],
            'bool_array': [True, False, True],
            'str_array': ['Hello', 'World'],
            'byte_array': [b'p', b'v'],
            'parameter_with_no_value': None
        }

    @staticmethod
    def get_modified_parameters():
        return {
            'bool_param': False,
            'int_param': 43,
            'double_param': 1.20,
            'str_param': 'Hello World',
            'int_array': [4, 5, 6],
            'double_array': [4.0, 5.0, 6.0],
            'bool_array': [False, True, False],
            'str_array': ['World', 'Hello'],
            'byte_array': [b'v', b'p'],
            'parameter_with_no_value': None
        }

    def __init__(self):
        super().__init__('param_node')
        parameters = ParamNode.get_node_parameters()
        for param_key, param_value in parameters.items():
            self.declare_parameter(param_key, param_value, ParameterDescriptor())


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
