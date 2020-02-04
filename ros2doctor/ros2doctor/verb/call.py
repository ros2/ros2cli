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

from argparse import ArgumentTypeError
import socket
import struct
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from ros2doctor.verb import VerbExtension

from std_msgs.msg import String

DEFAULT_GROUP = '225.0.0.1'
DEFAULT_PORT = 49150


def positive_int(string: str) -> int:
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value <= 0:
        raise ArgumentTypeError('value must be a positive integer')
    return value


class CallVerb(VerbExtension):
    """
    Check network connectivity between multiple hosts.

    This command can be invoked on multiple hosts to confirm that they can talk to each other
    by using talker/listener, multicast send/receive to check topic discovering and
    UDP communication.
    This command outputs a summary table of msgs statistics at a custom rate(Hz).
    """

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            'topic_name', nargs='?', default='/canyouhearme',
            help="Name of ROS topic to publish to (default: '/canyouhearme')")
        parser.add_argument(
            'time_period', nargs='?', default=0.1,
            help='Time period to publish/send one message (default: 0.1s)')
        parser.add_argument(
            'duration', nargs='?', default=20, type=positive_int,
            help='How long this process runs (default: 20s)')
        parser.add_argument(
            '-r', '--rate', metavar='N', type=float, default=1.0,
            help='Rate in Hz to print summary table (default: 1.0)')
        parser.add_argument(
            '--ttl', type=positive_int,
            help='TTL for multicast send (default: None)')

    def main(self, *, args):
        global summary_table
        summary_table = SummaryTable()
        rclpy.init()
        executor = SingleThreadedExecutor()
        pub_node = Talker(args.topic_name, args.time_period)
        sub_node = Listener(args.topic_name)
        executor.add_node(pub_node)
        executor.add_node(sub_node)
        try:
            prev_time = time.time()
            timeout = time.time() + args.duration
            # pub/sub thread
            exec_thread = threading.Thread(target=executor.spin)
            exec_thread.start()
            while time.time() < timeout:
                # print table at user determined rate
                if (time.time() - prev_time > float(1/args.rate)):
                    summary_table.format_print_summary(args.topic_name, args.rate)
                    summary_table.reset()
                    prev_time = time.time()
                # multicast threads
                send_thread = threading.Thread(target=_send, kwargs={'ttl': args.ttl})
                send_thread.daemon = True
                receive_thread = threading.Thread(target=_receive)
                receive_thread.daemon = True
                receive_thread.start()
                send_thread.start()
                time.sleep(args.time_period)
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            rclpy.shutdown()
            pub_node.destroy_node()
            sub_node.destroy_node()


class Talker(Node):
    """Initialize talker node."""

    def __init__(self, topic, time_period, *, qos=10):
        super().__init__('ros2doctor_talker')
        self.i = 0
        self.pub = self.create_publisher(String, topic, qos)
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        hostname = socket.gethostname()
        # publish
        msg.data = f"hello, it's me {hostname}"
        summary_table.increment_pub()
        self.pub.publish(msg)
        self.i += 1


class Listener(Node):
    """Initialize listener node."""

    def __init__(self, topic, *, qos=10):
        super().__init__('ros2doctor_listener')
        self.sub = self.create_subscription(
            String,
            topic,
            self.sub_callback,
            qos)

    def sub_callback(self, msg):
        # subscribe
        msg_data = msg.data.split()
        pub_hostname = msg_data[-1]
        if pub_hostname != socket.gethostname():
            summary_table.increment_sub(pub_hostname)


def _send(*, group=DEFAULT_GROUP, port=DEFAULT_PORT, ttl=None):
    """Multicast send one message."""
    hostname = socket.gethostname()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    if ttl is not None:
        packed_ttl = struct.pack('b', ttl)
        s.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, packed_ttl)
    try:
        s.sendto(f"hello, it's me {hostname}".encode('utf-8'), (group, port))
        summary_table.increment_send()
    finally:
        s.close()


def _receive(*, group=DEFAULT_GROUP, port=DEFAULT_PORT, timeout=None):
    """Multicast receive."""
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
            data, _ = s.recvfrom(4096)
            data = data.decode('utf-8')
            sender_hostname = data.split()[-1]
            if sender_hostname != socket.gethostname():
                summary_table.increment_receive(sender_hostname)
        finally:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
    finally:
        s.close()


class SummaryTable():
    """Summarize number of msgs published/sent and subscribed/received."""

    def __init__(self):
        """Initialize empty summary table."""
        self.lock = threading.Lock()
        self.pub = 0
        self.send = 0
        self.sub = {}
        self.receive = {}

    def reset(self):
        """Reset summary table to empty each time after printing."""
        self.pub = 0
        self.send = 0
        self.sub = {}
        self.receive = {}

    def increment_pub(self):
        """Increment published msg count."""
        self.lock.acquire()
        try:
            self.pub += 1
        finally:
            self.lock.release()

    def increment_sub(self, hostname):
        """Increment subscribed msg count from different host(s)."""
        self.lock.acquire()
        try:
            if hostname not in self.sub:
                self.sub[hostname] = 1
            else:
                self.sub[hostname] += 1
        finally:
            self.lock.release()

    def increment_send(self):
        """Increment multicast-sent msg count."""
        self.lock.acquire()
        try:
            self.send += 1
        finally:
            self.lock.release()

    def increment_receive(self, hostname):
        """Increment multicast-received msg count from different host(s)."""
        self.lock.acquire()
        try:
            if hostname not in self.receive:
                self.receive[hostname] = 1
            else:
                self.receive[hostname] += 1
        finally:
            self.lock.release()

    def format_print_summary(self, topic, rate, *, group=DEFAULT_GROUP, port=DEFAULT_PORT):
        """Print content in a table format."""
        def _format_print_summary_helper(table):
            msg_freq = 1/rate
            print('{:<15} {:<20} {:<10}'.format('', 'Hostname', f'Msg Count /{msg_freq}s'))
            for name, count in table.items():
                print('{:<15} {:<20} {:<10}'.format('', name, count))

        print('MULTIMACHINE COMMUNICATION SUMMARY')
        print(f'Topic: {topic}, Published Msg Count: {self.pub}')
        print('Subscribed from:')
        _format_print_summary_helper(self.sub)
        print(
            f'Multicast Group/Port: {DEFAULT_GROUP}/{DEFAULT_PORT}, '
            f'Sent Msg Count: {self.send}')
        print('Received from:')
        _format_print_summary_helper(self.receive)
        print('-'*60)
