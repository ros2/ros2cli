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

from multiprocessing import Process
import time
import socket
import struct

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from ros2doctor.verb import VerbExtension
from ros2multicast.api import send
from ros2multicast.api import receive

from std_msgs.msg import String

DEFAULT_GROUP = '225.0.0.1'
DEFAULT_PORT = 49150


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
            
            while True:
                p_receive = Process(target=udp_receive)
                p_send = Process(target=udp_send)
                executor.spin_once()
                p_receive.start()
                p_send.start()
                p_send.join()
                p_receive.join()
                p_send.terminate()
                p_receive.terminate()
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        executor.shutdown()
        caller_node.destroy_node()
        receiver_node.destroy_node()


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'ring', 10)
        time_period = 5
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
        self.sub = self.create_subscription(String,
            'ring',
            self.knock_callback,
            10)

    def knock_callback(self, msg):
        caller_hostname = msg.data.split()[-1]
        # if caller_hostname != socket.gethostname():
        self.get_logger().info(f'I heard from {caller_hostname} on ring')


def udp_send(*, group=DEFAULT_GROUP, port=DEFAULT_PORT):
    local_hostname = socket.gethostname()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        print('Sending one udp packet')
        s.sendto(f'Hello from {local_hostname}'.encode('utf-8'), (group, port))
    finally:
        s.close()


def udp_receive(*, group=DEFAULT_GROUP, port=DEFAULT_PORT, timeout=None):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            # not available on Windows
            pass
        s.bind(('', port))

        s.settimeout(timeout)

        mreq = struct.pack('4sl', socket.inet_aton(group), socket.INADDR_ANY)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        try:
            data, sender_addr = s.recvfrom(4096)
            data = data.decode('utf-8')
            sender_hostname = data.split()[-1]
            print(f'received one packet from {sender_hostname} on {sender_addr}')
        finally:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
    finally:
        s.close()

