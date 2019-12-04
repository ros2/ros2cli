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

import socket

import rclpy
from rclpy.node import Node
from ros2doctor.verb import VerbExtension

from std_msgs.msg import String


class CallVerb(VerbExtension):
    """Pub msg and hostname; listen on the same topic. Print Periodically."""

    def add_arguments(self, parser, cli_name):
        return NotImplementedError
    
    def main(self, *, args):
        rclpy.init(args=None)
        publish()

        rclpy.shutdown()


def publish():
    caller_node = Talker()

    try:
        rclpy.spin(caller_node)
    except KeyboardInterrupt:
        pass

    caller_node.destroy_node()


def subscribe():
    return NotImplementedError


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'knock', 10)
        time_period = 1.0
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        hostname = socket.gethostname()
        msg.data = f'{self.i}. Hello Eloquent from {hostname}'
        self.i += 1
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.pub.publish(msg)