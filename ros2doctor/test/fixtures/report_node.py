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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import SetBool


class ReportTestNode(Node):

    def __init__(self):
        super().__init__('talker_node')
        self.create_publisher(String, 'msg', 10)
        self.create_subscription(String, 'msg', lambda msg: None, 10)
        self.create_client(SetBool, 'baz')
        self.create_service(SetBool, 'bar', lambda req, res: res)


def main():
    try:
        with rclpy.init():
            node = ReportTestNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('report_test_node stopped cleanly')


if __name__ == '__main__':
    main()
