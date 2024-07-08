# Copyright 2021 Open Source Robotics Foundation, Inc.
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
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from std_msgs.msg import String


class ListenerNode(Node):

    def __init__(self):
        super().__init__('listener')
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.sub = self.create_subscription(
            String, 'chatter', self.callback, qos_profile
        )

    def callback(self, msg):
        self.get_logger().info('I heard: [%s]' % msg.data)


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = ListenerNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('listener stopped cleanly')


if __name__ == '__main__':
    main()
