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
from rclpy.executors import SingleThreadedExecutor
from ros2doctor.verb import VerbExtension

from std_msgs.msg import String


class CallVerb(VerbExtension):
    """Pub msg and hostname; listen on the same topic; print periodically."""

    def main(self, *, args):
        rclpy.init()
        caller_node = Talker()
        receiver_node = Listener()
        executor = SingleThreadedExecutor()
        executor.add_node(caller_node)
        executor.add_node(receiver_node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        caller_node.destroy_node()
        receiver_node.destroy_node()
        executor.shutdown()


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
        msg.data = f'{self.i}. Hello ROS2 from {hostname}'
        self.i += 1
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.pub.publish(msg)


class Listener(Node):
    
    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String, 'knock', self.knock_callback, 10)

    def knock_callback(self, msg):
        caller_hostname = msg.data.split()[-1]
        if caller_hostname != socket.gethostname():
            self.get_logger().info(f'I heard from {caller_hostname}')
        else:
            self.get_logger().info('I heard myself')
