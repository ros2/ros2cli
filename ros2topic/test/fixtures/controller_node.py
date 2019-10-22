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

from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')
        self.pub = self.create_publisher(
            TwistStamped, 'cmd_vel', qos_profile_system_default
        )
        self.tmr = self.create_timer(1.0, self.callback)

    def callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 1.0
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('controller stopped cleanly')
    except BaseException:
        print('exception in controller:', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
