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
import math

import rclpy

from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from ros2cli.node.direct import DirectNode
from ros2topic.api import get_msg_class
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension

DEFAULT_WINDOW_SIZE = 10000


def positive_int(string):
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value <= 0:
        raise ArgumentTypeError('value must be a positive integer')
    return value


class DelayVerb(VerbExtension):
    """Display delay of topic from timestamp in header."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'topic',
            help='Topic name to be calcurated the delay')
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')
        parser.add_argument(
            '--window', '-w', type=positive_int, default=DEFAULT_WINDOW_SIZE,
            help='window size, in # of messages, for calculating rate, '
                 'string to (default: %d)' % DEFAULT_WINDOW_SIZE)

    def main(self, *, args):
        return main(args)


def main(args):
    with DirectNode(args) as node:
        _rostopic_delay(
            node.node, args.topic, window_size=args.window)


class ROSTopicDelay(object):
    """Receives messages for a topic and computes timestamp delay."""

    def __init__(self, node, window_size):
        import threading
        self.lock = threading.Lock()
        self.last_msg_tn = 0
        self.msg_t0 = -1.
        self.msg_tn = 0
        self.delays = []

        self.window_size = window_size

        self._clock = node.get_clock()

    def callback_delay(self, msg):
        """
        Calculate delay time.

        :param msg: Message instance
        """
        if not hasattr(msg, 'header'):
            raise RuntimeError('msg does not have header')
        with self.lock:
            curr_rostime = self._clock.now()

            # time reset
            if curr_rostime.nanoseconds == 0:
                if len(self.delays) > 0:
                    print('time has reset, resetting counters')
                    self.delays = []
                return

            curr = curr_rostime.nanoseconds
            if self.msg_t0 < 0 or self.msg_t0 > curr:
                self.msg_t0 = curr
                self.msg_tn = curr
                self.delays = []
            else:
                # store the duration nanoseconds in self.delays
                duration = (curr_rostime - Time.from_msg(msg.header.stamp))
                self.delays.append(duration.nanoseconds)
                self.msg_tn = curr

            if len(self.delays) > self.window_size:
                self.delays.pop(0)

    def get_delay(self):
        """
        Calculate the average publising delay.

        :returns: tuple of stat results
            (rate, min_delta, max_delta, standard deviation, window number)
            None when waiting for the first message or there is no new one
        """
        if self.msg_tn == self.last_msg_tn:
            return
        with self.lock:
            if not self.delays:
                return
            n = len(self.delays)

            mean = sum(self.delays) / n
            std_dev = math.sqrt(sum((x - mean)**2 for x in self.delays) / n)

            max_delta = max(self.delays)
            min_delta = min(self.delays)

            self.last_msg_tn = self.msg_tn
        return mean, min_delta, max_delta, std_dev, n

    def print_delay(self):
        """Print the average publishing delay to screen."""
        if not self.delays:
            return
        ret = self.get_delay()
        if ret is None:
            print('no new messages')
            return
        delay, min_delta, max_delta, std_dev, window = ret
        # convert nanoseconds to seconds when print
        print('average delay: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s'
              % (delay * 1e-9, min_delta * 1e-9, max_delta * 1e-9, std_dev * 1e-9, window))


def _rostopic_delay(node, topic, window_size=DEFAULT_WINDOW_SIZE):
    """
    Periodically print the publishing delay of a topic to console until shutdown.

    :param topic: topic name, ``str``
    :param window_size: number of messages to average over, ``unsigned_int``
    :param blocking: pause delay until topic is published, ``bool``
    """
    # pause hz until topic is published
    msg_class = get_msg_class(node, topic, blocking=True, include_hidden_topics=True)

    if msg_class is None:
        node.destroy_node()
        return

    rt = ROSTopicDelay(node, window_size)
    node.create_subscription(
        msg_class,
        topic,
        rt.callback_delay,
        qos_profile_sensor_data)

    timer = node.create_timer(1, rt.print_delay)
    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()
