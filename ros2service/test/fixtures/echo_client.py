# Copyright 2022 Open Source Robotics Foundation, Inc.
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
from rclpy.node import Node

from test_msgs.srv import BasicTypes


class EchoClient(Node):

    def __init__(self):
        super().__init__('echo_client')
        self.future = None
        self.client = self.create_client(BasicTypes, 'echo')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('echo service not available, waiting again...')
        self.req = BasicTypes.Request()

    def send_request(self):
        print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
        print("send_request")
        self.req.string_value = "test"
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)
    node = EchoClient()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
