# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# This file is originally from:
# https://github.com/ros/ros_comm/blob/6e5016f4b2266d8a60c9a1e163c4928b8fc7115e/tools/rostopic/src/rostopic/__init__.py

from argparse import ArgumentTypeError
import threading
import time
import traceback

import rclpy
from rclpy.qos import qos_profile_sensor_data
from ros2cli.node.direct import DirectNode
from ros2topic.api import get_msg_class
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension

DEFAULT_WINDOW_SIZE = 100


def positive_int(string):
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value <= 0:
        raise ArgumentTypeError('value must be a positive integer')
    return value


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
            help='window size, in # of messages, for calculating rate '
                 '(default: %d)' % DEFAULT_WINDOW_SIZE, metavar='WINDOW')

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

    def callback(self, data):
        """Execute ros sub callback."""
        with self.lock:
            try:
                t = time.monotonic()
                self.times.append(t)
                # TODO(yechun1): Subscribing to the msgs and calculate the length may be
                # inefficiency. To optimize here if found better solution.
                self.sizes.append(len(data))  # AnyMsg instance
                assert(len(self.times) == len(self.sizes))

                if len(self.times) > self.window_size:
                    self.times.pop(0)
                    self.sizes.pop(0)
            except Exception:
                traceback.print_exc()

    def print_bw(self):
        """Print the average publishing bw to screen."""
        if len(self.times) < 2:
            return
        with self.lock:
            n = len(self.times)
            tn = time.monotonic()
            t0 = self.times[0]

            total = sum(self.sizes)
            bytes_per_s = total / (tn - t0)
            mean = total / n

            # min and max
            max_s = max(self.sizes)
            min_s = min(self.sizes)

        # min/max and even mean are likely to be much smaller,
        # but for now I prefer unit consistency
        if bytes_per_s < 1000:
            bw, mean, min_s, max_s = ['%.2fB/s' % v for v in [bytes_per_s, mean, min_s, max_s]]
        elif bytes_per_s < 1000000:
            bw, mean, min_s, max_s = \
                ['%.2fKB/s' % (v / 1000) for v in [bytes_per_s, mean, min_s, max_s]]
        else:
            bw, mean, min_s, max_s = \
                ['%.2fMB/s' % (v / 1000000) for v in [bytes_per_s, mean, min_s, max_s]]

        print('average: %s\n\tmean: %s min: %s max: %s window: %s' % (bw, mean, min_s, max_s, n))


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

    print('Subscribed to [%s]' % topic)
    timer = node.create_timer(1, rt.print_bw)
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()
