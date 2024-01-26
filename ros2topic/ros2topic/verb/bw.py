# Copyright (c) 2008, Willow Garage, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This file is originally from:
# https://github.com/ros/ros_comm/blob/6e5016f4b2266d8a60c9a1e163c4928b8fc7115e/tools/rostopic/src/rostopic/__init__.py

import sys
import threading
import traceback

import rclpy
from rclpy.qos import qos_profile_sensor_data
from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode
from ros2topic.api import get_msg_class
from ros2topic.api import positive_int
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension

DEFAULT_WINDOW_SIZE = 100


def str_bytes(num_bytes):
    return f'{num_bytes:.0f} B'


def str_kilobytes(num_bytes):
    return f'{num_bytes/1000:.2f} KB'


def str_megabytes(num_bytes):
    return f'{num_bytes/1000/1000:.2f} MB'


class BwVerb(VerbExtension):
    """Display bandwidth used by topic."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'topic',
            help='Topic name to monitor for bandwidth utilization')
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')
        parser.add_argument(
            '--window', '-w', type=positive_int, default=DEFAULT_WINDOW_SIZE,
            help='maximum window size, in # of messages, for calculating rate '
                 f'(default: {DEFAULT_WINDOW_SIZE})', metavar='WINDOW')
        add_direct_node_arguments(parser)

    def main(self, *, args):
        with DirectNode(args) as node:
            _rostopic_bw(node.node, args.topic, window_size=args.window)


class ROSTopicBandwidth(object):

    def __init__(self, node, window_size):
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.sizes = []
        self.times = []
        self.window_size = window_size
        self.use_sim_time = node.get_parameter('use_sim_time').value
        self.clock = node.get_clock()

    def callback(self, data):
        """Execute ros sub callback."""
        with self.lock:
            try:
                t = self.clock.now()
                self.times.append(t)
                # TODO(yechun1): Subscribing to the msgs and calculate the length may be
                # inefficient. Optimize here if a better solution is found.
                self.sizes.append(len(data))  # AnyMsg instance
                assert len(self.times) == len(self.sizes)

                if len(self.times) > self.window_size:
                    self.times.pop(0)
                    self.sizes.pop(0)
            except Exception:
                traceback.print_exc()

    def get_bw(self):
        """Get the average publishing bw."""
        if len(self.times) < 2:
            return None, None, None, None, None
        with self.lock:
            n = len(self.times)
            tn = self.clock.now()
            t0 = self.times[0]
            if tn <= t0:
                print('WARNING: time is reset!', file=sys.stderr)
                self.times = []
                self.sizes = []
                return None, None, None, None, None

            total = sum(self.sizes)
            bytes_per_s = total / ((tn.nanoseconds - t0.nanoseconds) * 1.e-9)
            mean = total / n

            # min and max
            max_s = max(self.sizes)
            min_s = min(self.sizes)

        return bytes_per_s, n, mean, min_s, max_s

    def print_bw(self):
        """Print the average publishing bw to screen."""
        (bytes_per_s, n, mean, min_s, max_s) = self.get_bw()
        if bytes_per_s is None:
            return

        # min/max and even mean are likely to be much smaller,
        # but for now I prefer unit consistency
        if bytes_per_s < 1000:
            bw, mean, min_s, max_s = map(str_bytes, (bytes_per_s, mean, min_s, max_s))
        elif bytes_per_s < 1000000:
            bw, mean, min_s, max_s = map(str_kilobytes, (bytes_per_s, mean, min_s, max_s))
        else:
            bw, mean, min_s, max_s = map(str_megabytes, (bytes_per_s, mean, min_s, max_s))

        # Bandwidth is per second
        bw += '/s'

        print(f'{bw} from {n} messages\n\tMessage size mean: {mean} min: {min_s} max: {max_s}')


def _rostopic_bw(node, topic, window_size=DEFAULT_WINDOW_SIZE):
    """Periodically print the received bandwidth of a topic to console until shutdown."""
    # pause bw until topic is published
    msg_class = get_msg_class(node, topic, blocking=True, include_hidden_topics=True)
    if msg_class is None:
        node.destroy_node()
        return

    rt = ROSTopicBandwidth(node, window_size)
    node.create_subscription(
        msg_class,
        topic,
        rt.callback,
        qos_profile_sensor_data,
        raw=True
    )

    print(f'Subscribed to [{topic}]')
    timer = node.create_timer(1, rt.print_bw)
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()
