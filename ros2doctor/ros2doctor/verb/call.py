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
import time
import threading
import struct

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros2doctor.verb import VerbExtension

from std_msgs.msg import String

DEFAULT_GROUP = '225.0.0.1'
DEFAULT_PORT = 49150
summary_table = {'topic_pub': 0,
                 'topic_sub': {},
                 'multicast_send': 0,
                 'multicast_receive': {}}


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'ring', 10)
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        hostname = socket.gethostname()
        # publish
        msg.data = f'publishing hello ROS2 from {hostname}'
        summary_table['topic_pub'] += 1
        # self.get_logger().info(f'Publishing: "{msg.data}"')
        self.pub.publish(msg)
        self.i += 1


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String,
                                            'ring',
                                            self.sub_callback,
                                            10)

    def sub_callback(self, msg):
        # subscribe
        msg_data = msg.data.split()
        caller_hostname = msg_data[-1]
        # if caller_hostname != socket.gethostname():
        if caller_hostname not in summary_table['topic_sub']:
            summary_table['topic_sub'][caller_hostname] = 1
        else:
            summary_table['topic_sub'][caller_hostname] += 1
        # print(msg.data)


def send():
    hostname = socket.gethostname()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        # print('Sending one udp packet')
        summary_table['multicast_send'] += 1
        s.sendto(f'Multicast hello from {hostname}'.encode('utf-8'), (DEFAULT_GROUP, DEFAULT_PORT))
    finally:
        s.close()


def receive():
    # multicast receive
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            # not available on Windows
            pass
        s.bind(('', DEFAULT_PORT))
    
        s.settimeout(None)

        mreq = struct.pack('4sl', socket.inet_aton(DEFAULT_GROUP), socket.INADDR_ANY)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        try:
            data, _ = s.recvfrom(4096)
            data = data.decode('utf-8')
            sender_hostname = data.split()[-1]
            if sender_hostname not in summary_table['multicast_receive']:
                summary_table['multicast_receive'][sender_hostname] = 1
            else:
                summary_table['multicast_receive'][sender_hostname] += 1
            # print(data)
        finally:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
    finally:
        s.close()


class CallVerb(VerbExtension):
    """Pub msg and hostname; listen on the same topic; print periodically."""

    def main(self, *, args):
        rclpy.init()
        pub_node = Talker()
        sub_node = Listener()
       
        executor = MultiThreadedExecutor()
        executor.add_node(pub_node)
        executor.add_node(sub_node)
        try:
            count = 0
            while True:
                if (count % 20 == 0 and count != 0):
                    print(summary_table)
                    summary_table['topic_pub'] = 0
                    summary_table['topic_sub'] = {}
                    summary_table['multicast_send'] = 0
                    summary_table['multicast_receive'] = {}
                    time.sleep(1)
                # pub/sub threads
                executor.spin_once()
                executor.spin_once()
                # multicast threads
                send_thread = threading.Thread(target=send, args=())
                send_thread.daemon = True
                receive_thread = threading.Thread(target=receive, args=())
                receive_thread.daemon = True
                receive_thread.start()
                send_thread.start()
                count += 1
                time.sleep(0.1)
        except KeyboardInterrupt:
            executor.shutdown()
            pub_node.destroy_node()
            sub_node.destroy_node()

