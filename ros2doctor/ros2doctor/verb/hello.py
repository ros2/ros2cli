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
import re
import socket
import struct
import threading

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_system_default
from ros2cli.node.direct import DirectNode
from ros2doctor.verb import VerbExtension
from std_msgs.msg import String

DEFAULT_GROUP = '225.0.0.1'
DEFAULT_PORT = 49150

NODE_NAME_PREFIX = \
    f"ros2doctor_{re.sub(r'[^0-9a-zA-Z_]', '_', socket.gethostname())}_{os.getpid()}"


def positive(type_):
    def _coerce(string):
        try:
            value = type_(string)
        except ValueError:
            value = -1
        if value <= 0:
            raise ArgumentTypeError('value must be a positive {type_.__name__}')
        return value
    return _coerce


positive_float = positive(float)
positive_int = positive(int)


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
            '-ep', '--emit-period', metavar='N', type=positive_float, default=0.1,
            help='Time period to publish/send one message (default: 0.1s)')
        parser.add_argument(
            '-pp', '--print-period', metavar='N', type=positive_float, default=1.0,
            help='Time period to print summary table (default: 1.0s)')
        parser.add_argument(
            '--ttl', type=positive_int,
            help='TTL for multicast send (default: None)')
        parser.add_argument(
            '-1', '--once', action='store_true', default=False,
            help='Publish and multicast send for one emit period then exit; used in test case.')

    def main(self, *, args, summary_table=None):
        if summary_table is None:
            summary_table = SummaryTable()
        with DirectNode(args, node_name=NODE_NAME_PREFIX + '_node') as node:
            publisher = HelloPublisher(node, args.topic, summary_table)
            HelloSubscriber(node, args.topic, summary_table)
            sender = HelloMulticastUDPSender(summary_table, ttl=args.ttl)
            receiver = HelloMulticastUDPReceiver(summary_table)
            receiver_thread = threading.Thread(target=receiver.recv)
            receiver_thread.start()

            executor = SingleThreadedExecutor()
            executor.add_node(node.node)

            executor_thread = threading.Thread(target=executor.spin)
            executor_thread.start()

            try:
                clock = node.get_clock()
                prev_time = clock.now()
                print_period = Duration(seconds=args.print_period)
                emit_rate = node.create_rate(frequency=1.0 / args.emit_period, clock=clock)
                while rclpy.ok():
                    current_time = clock.now()
                    if (current_time - prev_time) > print_period:
                        summary_table.format_print_summary(args.topic, args.print_period)
                        summary_table.reset()
                        prev_time = current_time
                    publisher.publish()
                    sender.send()
                    emit_rate.sleep()
                    if args.once:
                        summary_table.format_print_summary(args.topic, args.print_period)
                        break
            except KeyboardInterrupt:
                pass
            finally:
                executor.shutdown()
                executor_thread.join()
                receiver.shutdown()
                receiver_thread.join()
                sender.shutdown()


class HelloPublisher:
    """Publish 'hello' messages over an std_msgs/msg/String topic."""

    def __init__(self, node, topic, summary_table, *, qos=qos_profile_system_default):
        self._summary_table = summary_table
        self._pub = node.create_publisher(String, topic, qos)

    def publish(self):
        msg = String()
        hostname = socket.gethostname()
        msg.data = f"hello, it's me {hostname}"
        self._summary_table.increment_pub()
        self._pub.publish(msg)


class HelloSubscriber:
    """Subscribe to 'hello' messages over an std_msgs/msg/String topic."""

    def __init__(self, node, topic, summary_table, *, qos=qos_profile_system_default):
        self._summary_table = summary_table
        self._sub = node.create_subscription(String, topic, self._callback, qos)

    def _callback(self, msg):
        msg_data = msg.data.split()
        pub_hostname = msg_data[-1]
        if pub_hostname != socket.gethostname():
            self._summary_table.increment_sub(pub_hostname)


class HelloMulticastUDPSender:
    """Send 'hello' messages over a multicast UDP socket."""

    def __init__(self, summary_table, group=DEFAULT_GROUP, port=DEFAULT_PORT, ttl=None):
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            if ttl is not None:
                packed_ttl = struct.pack('b', ttl)
                self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, packed_ttl)
        except Exception:
            self._socket.close()
            raise
        self._summary_table = summary_table
        self._group = group
        self._port = port

    def send(self):
        hostname = socket.gethostname()
        self._socket.sendto(
            f"hello, it's me {hostname}".encode('utf-8'), (self._group, self._port)
        )
        self._summary_table.increment_send()

    def shutdown(self):
        self._socket.close()


class HelloMulticastUDPReceiver:
    """Receive 'hello' messages over a multicast UDP socket."""

    def __init__(self, summary_table, group=DEFAULT_GROUP, port=DEFAULT_PORT, timeout=None):
        self._dummy_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        try:
            self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except AttributeError:
                # not available on Windows
                pass
            self._socket.bind(('', port))

            self._socket.settimeout(timeout)

            self._mreq = struct.pack('4sl', socket.inet_aton(group), socket.INADDR_ANY)
            self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, self._mreq)
        except Exception:
            self._dummy_socket.close()
            self._socket.close()
            raise
        self._is_shutdown = False
        self._summary_table = summary_table
        self._group = group
        self._port = port

    def recv(self):
        try:
            while not self._is_shutdown:
                data, _ = self._socket.recvfrom(4096)
                data = data.decode('utf-8')
                sender_hostname = data.split()[-1]
                if sender_hostname != socket.gethostname():
                    self._summary_table.increment_receive(sender_hostname)
        except socket.timeout:
            pass

    def shutdown(self):
        if self._is_shutdown:
            return
        self._is_shutdown = True
        self._dummy_socket.sendto(
            f'{socket.gethostname()}'.encode('utf-8'), ('', self._port)
        )
        self._dummy_socket.close()
        self._socket.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, self._mreq)
        self._socket.close()


class SummaryTable:
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
