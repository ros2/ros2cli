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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from test_msgs.srv import BasicTypes


class EchoServer(Node):

    def __init__(self):
        super().__init__('echo_server')
        self.server = self.create_service(BasicTypes, 'echo', self.callback)

    def callback(self, request, response):
        for field_name in request.get_fields_and_field_types():
            setattr(response, field_name, getattr(request, field_name))
        return response


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = EchoServer()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('server stopped cleanly')


if __name__ == '__main__':
    main()
