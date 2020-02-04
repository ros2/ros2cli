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
                send_thread = threading.Thread(target=send, args=())
                send_thread.daemon = True
                receive_thread = threading.Thread(target=receive, args=())
                receive_thread.daemon = True
                receive_thread.start()
                send_thread.start()
                count += 1
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            rclpy.shutdown()
            pub_node.destroy_node()
            sub_node.destroy_node()


class Talker(Node):
    """Initialize talker node."""

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, DEFAULT_TOPIC, 10)
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        hostname = socket.gethostname()
        # publish
        msg.data = f'Publish hello from {hostname}'
        summary_table['pub'] += 1
        self.pub.publish(msg)
        self.i += 1


class Listener(Node):
    """Initialize listener node."""

    def __init__(self):
        super().__init__('listener')
        self.sub = self.create_subscription(String,
                                            DEFAULT_TOPIC,
                                            self.sub_callback,
                                            10)

    def sub_callback(self, msg):
        # subscribe
        msg_data = msg.data.split()
        caller_hostname = msg_data[-1]
        if caller_hostname != socket.gethostname():
            if caller_hostname not in summary_table['sub']:
                summary_table['sub'][caller_hostname] = 1
            else:
                summary_table['sub'][caller_hostname] += 1


<<<<<<< HEAD
def send():
    """Multicast send."""
=======
def _send(*, group=DEFAULT_GROUP, port=DEFAULT_PORT, ttl=None):
    """Multicast send one message."""
>>>>>>> d283d3a... add summary table doc string
    hostname = socket.gethostname()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    try:
        summary_table['send'] += 1
        s.sendto(f'Multicast hello from {hostname}'.encode('utf-8'), (DEFAULT_GROUP, DEFAULT_PORT))
    finally:
        s.close()


def receive():
    """Multicast receive."""
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
            if sender_hostname != socket.gethostname():
                if sender_hostname not in summary_table['receive']:
                    summary_table['receive'][sender_hostname] = 1
                else:
                    summary_table['receive'][sender_hostname] += 1
        finally:
            s.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
    finally:
        s.close()


<<<<<<< HEAD
def _spawn_summary_table():
    """Spawn summary table with new content after each print."""
    summary_table['pub'] = 0
    summary_table['sub'] = {}
    summary_table['send'] = 0
    summary_table['receive'] = {}


def _format_print_helper(table):
    """Format summary table."""
    print('{:<15} {:<20} {:<10}'.format('', 'Hostname', 'Msg Count /2s'))
    for name, count in table.items():
        print('{:<15} {:<20} {:<10}'.format('', name, count))


def format_print(summary_table):
    """Print content in summary table."""
    pub_count = summary_table['pub']
    send_count = summary_table['send']
    print('MULTIMACHINE COMMUNICATION SUMMARY')
    print(f'Topic: {DEFAULT_TOPIC}, Published Msg Count: {pub_count}')
    print('Subscribed from:')
    _format_print_helper(summary_table['sub'])
    print(f'Multicast Group/Port: {DEFAULT_GROUP}/{DEFAULT_PORT}, Sent Msg Count: {send_count}')
    print('Received from:')
    _format_print_helper(summary_table['receive'])
    print('-'*60)
=======
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
            if hostname not in self.receive:
                self.receive[hostname] = 1
            else:
                self.receive[hostname] += 1
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
            if hostname not in self.sub:
                self.sub[hostname] = 1
            else:
                self.sub[hostname] += 1
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
>>>>>>> d283d3a... add summary table doc string
