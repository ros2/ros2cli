# Copyright 2026 Open Source Robotics Foundation, Inc.
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


import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ParamListHangNode(Node):
    """A node that hangs on list_parameters service calls."""

    def __init__(self):
        super().__init__('param_list_hang_node')
        # Sleep indefinitely to simulate a non-responsive service
        while rclpy.ok():
            time.sleep(1.0)


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = ParamListHangNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('param_list_hang_node stopped cleanly')


if __name__ == '__main__':
    main()
