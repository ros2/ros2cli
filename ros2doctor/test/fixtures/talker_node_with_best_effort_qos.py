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


class TalkerNode(Node):

    def __init__(self):
        super().__init__('talker_node')
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.count = 1
        self.pub = self.create_publisher(String, 'chatter', qos_profile=qos_profile)
        self.tmr = self.create_timer(1.0, self.callback)

    def callback(self):
        self.pub.publish(String(data='Hello World: {0}'.format(self.count)))
        self.count += 1


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = TalkerNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('talker stopped cleanly')


if __name__ == '__main__':
    main()
