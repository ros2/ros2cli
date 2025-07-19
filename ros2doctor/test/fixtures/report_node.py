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

import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool
from test_msgs.action import Fibonacci


class ReportTestNode(Node):

    def __init__(self):
        # Disable all extraneous node services to simplify the test
        super().__init__('report_node', start_parameter_services=False)
        self.create_publisher(String, 'msg', 10)
        self.create_subscription(String, 'msg', lambda msg: None, 10)
        self.create_client(SetBool, 'baz')
        self.create_service(SetBool, 'bar', lambda req, res: res)
        ActionServer(self, Fibonacci, 'fibonacci', lambda handle: Fibonacci.Result())
        ActionClient(self, Fibonacci, 'fibonacci')


def main():
    try:
        with rclpy.init():
            node = ReportTestNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('report_test_node stopped cleanly')


if __name__ == '__main__':
    main()
