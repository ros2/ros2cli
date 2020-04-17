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
import os
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


class HelloVerb(VerbExtension):
    """
    Check network connectivity between multiple hosts.

    This command can be invoked on multiple hosts to confirm that they can talk to each other
    by using talker/listener, multicast send/receive to check topic discovering and
    UDP communication.
    This command outputs a summary table of msgs statistics at a custom period(s).
    """

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-t', '--topic', nargs='?', default='/canyouhearme',
            help="Name of ROS topic to publish to (default: '/canyouhearme')")
        parser.add_argument(
            '-ep', '--emit-period', metavar='N', type=float, default=0.1,
            help='Time period to publish/send one message (default: 0.1s)')
        parser.add_argument(
            '-pp', '--print-period', metavar='N', type=float, default=1.0,
            help='Time period to print summary table (default: 1.0s)')
        parser.add_argument(
            '--ttl', type=positive_int,
            help='TTL for multicast send (default: None)')
        parser.add_argument(
            '-1', '--once', action='store_true', default=False,
            help='Publish and multicast send for one emit period then exit; used in test case.')

    def main(self, *, args):
        global summary_table
        summary_table = SummaryTable()
        rclpy.init()
        executor = SingleThreadedExecutor()
        pub_node = Talker(args.topic, args.emit_period)
        sub_node = Listener(args.topic)
        executor.add_node(pub_node)
        executor.add_node(sub_node)
        try:
            prev_time = time.time()
            # pub/sub thread
            exec_thread = threading.Thread(target=executor.spin)
            exec_thread.start()
            while True:
                if (time.time() - prev_time > args.print_period):
                    summary_table.format_print_summary(args.topic, args.print_period)
                    summary_table.reset()
                    prev_time = time.time()
                # multicast threads
                send_thread = threading.Thread(target=_send, kwargs={'ttl': args.ttl})
                send_thread.daemon = True
                receive_thread = threading.Thread(target=_receive)
                receive_thread.daemon = True
                receive_thread.start()
                send_thread.start()
                time.sleep(args.emit_period)
                if args.once:
                    return summary_table
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
        node_name = 'ros2doctor_' + socket.gethostname() + str(os.getpid()) + '_talker'
        super().__init__(node_name)
        self._i = 0
        self._pub = self.create_publisher(String, topic, qos)
        self._timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        hostname = socket.gethostname()
        msg.data = f"hello, it's me {hostname}"
        summary_table.increment_pub()
        self._pub.publish(msg)
        self._i += 1


class Listener(Node):
    """Initialize listener node."""

    def __init__(self, topic, *, qos=10):
        node_name = 'ros2doctor_' + socket.gethostname() + str(os.getpid()) + '_listener'
        super().__init__(node_name)
        self._sub = self.create_subscription(
            String,
            topic,
            self.sub_callback,
            qos)

    def sub_callback(self, msg):
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
        self._pub = 0
        self._send = 0
        self._sub = {}
        self._receive = {}

    def reset(self):
        """Reset summary table to empty each time after printing."""
        with self.lock:
            self._pub = 0
            self._send = 0
            self._sub = {}
            self._receive = {}

    def increment_pub(self):
        """Increment published msg count."""
        with self.lock:
            self._pub += 1

    def increment_sub(self, hostname):
        """Increment subscribed msg count from different host(s)."""
        with self.lock:
            if hostname not in self._sub:
                self._sub[hostname] = 1
            else:
                self._sub[hostname] += 1

    def increment_send(self):
        """Increment multicast-sent msg count."""
        with self.lock:
            self._send += 1

    def increment_receive(self, hostname):
        """Increment multicast-received msg count from different host(s)."""
        with self.lock:
            if hostname not in self._receive:
                self._receive[hostname] = 1
            else:
                self._receive[hostname] += 1

    def format_print_summary(self, topic, print_period, *, group=DEFAULT_GROUP, port=DEFAULT_PORT):
        """Print content in a table format."""
        def _format_print_summary_helper(table):
            print('{:<15} {:<20} {:<10}'.format('', 'Hostname', f'Msg Count /{print_period}s'))
            for name, count in table.items():
                print('{:<15} {:<20} {:<10}'.format('', name, count))

        print('MULTIMACHINE COMMUNICATION SUMMARY')
        print(f'Topic: {topic}, Published Msg Count: {self._pub}')
        print('Subscribed from:')
        _format_print_summary_helper(self._sub)
        print(
            f'Multicast Group/Port: {group}/{port}, '
            f'Sent Msg Count: {self._send}')
        print('Received from:')
        _format_print_summary_helper(self._receive)
        print('-'*60)
