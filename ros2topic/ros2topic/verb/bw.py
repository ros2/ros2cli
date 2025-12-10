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

from collections import defaultdict

import functools
import sys
import threading
import time
import traceback

import rclpy

from rclpy.executors import ExternalShutdownException

from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode
from ros2cli.qos import add_qos_arguments
from ros2cli.qos import choose_qos

from ros2topic.api import get_msg_class
from ros2topic.api import get_topic_names_and_types
from ros2topic.api import positive_int
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension
from ros2topic.verb.echo import clear_terminal

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
        parser.description = (
            'Display bandwidth used by topic.\n\n'
            'note:\n'
            '  This bandwidth reflects the receiving rate on subscription, '
            'which might be affected by platform resources and QoS configuration, '
            "and may not exactly match the publisher's bandwidth."
        )
        arg = parser.add_argument(
            'topic_name',
            nargs='*',
            help="Names of the ROS topics to monitor for bandwidth utilization (e.g. '/chatter')")
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')
        parser.add_argument(
            '--all', '-a',
            dest='all_topics', default=False, action='store_true',
            help='subscribe to all available topics')
        add_qos_arguments(parser, 'subscribe', 'sensor_data')
        parser.add_argument(
            '--window', '-w', dest='window_size', type=positive_int, default=DEFAULT_WINDOW_SIZE,
            help='maximum window size, in # of messages, for calculating rate '
                 f'(default: {DEFAULT_WINDOW_SIZE})', metavar='WINDOW')
        add_direct_node_arguments(parser)

    def main(self, *, args):
        return main(args)


def main(args):
    if not args.all_topics and not args.topic_name:
        raise RuntimeError('Either specify topic names or use --all/-a option')
    if args.all_topics and args.topic_name:
        raise RuntimeError('Cannot specify both --all/-a and topic names')

    topics = args.topic_name

    with DirectNode(args) as node:
        # Get all available topics at this moment
        if args.all_topics:
            topic_names_and_types = get_topic_names_and_types(
                node=node.node,
                include_hidden_topics=args.include_hidden_topics)
            topics = [name for name, _ in topic_names_and_types]
            if not topics:
                print('No topics available')
                return
            print(f'Subscribing to all {len(topics)} available topics...')
        return _rostopic_bw(node.node, topics, qos_args=args, window_size=args.window_size,
                            all_topics=args.all_topics)


class ROSTopicBandwidth(object):

    def __init__(self, node, window_size):
        self.lock = threading.Lock()
        self._last_printed_tn = defaultdict(int)
        self._sizes = defaultdict(list)
        self._times = defaultdict(list)
        self.window_size = window_size
        self._clock = node.get_clock()

    def get_last_printed_tn(self, topic=None):
        return self._last_printed_tn[topic]

    def set_last_printed_tn(self, value, topic=None):
        self._last_printed_tn[topic] = value

    def get_sizes(self, topic=None):
        return self._sizes[topic]

    def set_sizes(self, value, topic=None):
        self._sizes[topic] = value

    def get_times(self, topic=None):
        return self._times[topic]

    def set_times(self, value, topic=None):
        self._times[topic] = value

    def callback(self, data, topic=None):
        """Execute ros sub callback."""
        with self.lock:
            try:
                t = self._clock.now()
                self.get_times(topic=topic).append(t)
                # TODO(yechun1): Subscribing to the msgs and calculate the length may be
                # inefficient. Optimize here if a better solution is found.
                self.get_sizes(topic=topic).append(len(data))  # AnyMsg instance
                assert len(self.get_times(topic=topic)) == len(self.get_sizes(topic=topic))

                if len(self.get_times(topic=topic)) > self.window_size:
                    self.get_times(topic=topic).pop(0)
                    self.get_sizes(topic=topic).pop(0)
            except Exception:
                traceback.print_exc()

    def get_bw(self, topic=None):
        """Get the average publishing bw."""
        with self.lock:
            times = self.get_times(topic=topic)
            sizes = self.get_sizes(topic=topic)
            if len(times) < 2:
                return None, None, None, None, None
            if topic is not None:
                last_time = times[-1]
                if last_time == self.get_last_printed_tn(topic=topic):
                    return None, None, None, None, None
            n = len(times)
            tn = self._clock.now()
            t0 = times[0]
            if tn <= t0:
                print('WARNING: time is reset!', file=sys.stderr)
                self.set_times([], topic=topic)
                self.set_sizes([], topic=topic)
                return None, None, None, None, None

            total = sum(sizes)
            bytes_per_s = total / ((tn.nanoseconds - t0.nanoseconds) * 1.e-9)
            mean = total / n

            # min and max
            max_s = max(sizes)
            min_s = min(sizes)

            if topic is not None:
                self.set_last_printed_tn(times[-1], topic=topic)

        return bytes_per_s, n, mean, min_s, max_s

    def print_bw(self, topics=None, clear_screen=False):
        """Print the average publishing bw to screen."""
        def get_format_bw(stat):
            bytes_per_s, n, mean, min_s, max_s = stat
            # min/max and even mean are likely to be much smaller,
            # but for now I prefer unit consistency
            if bytes_per_s < 1000:
                bw, mean_str, min_str, max_str = map(
                    str_bytes, (bytes_per_s, mean, min_s, max_s))
            elif bytes_per_s < 1000000:
                bw, mean_str, min_str, max_str = map(
                    str_kilobytes, (bytes_per_s, mean, min_s, max_s))
            else:
                bw, mean_str, min_str, max_str = map(
                    str_megabytes, (bytes_per_s, mean, min_s, max_s))
            # Bandwidth is per second
            bw += '/s'
            return bw, mean_str, min_str, max_str, n

        # Clear screen if requested (useful when monitoring all topics)
        if clear_screen:
            clear_terminal()

        if topics is None or len(topics) == 1:
            topic = topics[0] if topics else None
            ret = self.get_bw(topic)
            if ret[0] is None:
                return
            bw, mean_str, min_str, max_str, n = get_format_bw(ret)
            print(f'{bw} from {n} messages\n\tMessage size mean: {mean_str} ' +
                  f'min: {min_str} max: {max_str}')
            return

        # monitoring multiple topics' bw
        header = ['topic', 'bandwidth', 'window', 'mean', 'min', 'max']
        stats = {h: [] for h in header}
        for topic in topics:
            bw_stat = self.get_bw(topic)
            if bw_stat[0] is None:
                continue
            bw, mean_str, min_str, max_str, n = get_format_bw(bw_stat)
            stats['topic'].append(topic)
            stats['bandwidth'].append(bw)
            stats['window'].append(str(n))
            stats['mean'].append(mean_str)
            stats['min'].append(min_str)
            stats['max'].append(max_str)
        if not stats['topic']:
            return
        print(_get_ascii_table(header, stats))


def _get_ascii_table(header, cols):
    # compose table with left alignment
    if not cols or not cols[header[0]]:
        return ''
    header_aligned = []
    col_widths = []
    for h in header:
        col_width = max(len(h), max(len(el) for el in cols[h]))
        col_widths.append(col_width)
        header_aligned.append(h.center(col_width))
        for i, el in enumerate(cols[h]):
            cols[h][i] = str(cols[h][i]).ljust(col_width)
    # sum of col and each 3 spaces width
    table_width = sum(col_widths) + 3 * (len(header) - 1)
    n_rows = len(cols[header[0]])
    body = '\n'.join('   '.join(cols[h][i] for h in header) for i in range(n_rows))
    table = '{header}\n{hline}\n{body}\n'.format(
        header='   '.join(header_aligned), hline='=' * table_width, body=body)
    return table


def _rostopic_bw(node, topics, qos_args, window_size=DEFAULT_WINDOW_SIZE, all_topics=False):
    """Periodically print the received bandwidth of topics to console until shutdown."""
    # pause bw until topic is published
    rt = ROSTopicBandwidth(node, window_size)
    topics_to_be_removed = []
    for topic in topics:
        msg_class = get_msg_class(
            node, topic, blocking=True, include_hidden_topics=True)

        if msg_class is None:
            topics_to_be_removed.append(topic)
            print('WARNING: failed to find message type for topic [%s]' % topic)
            continue

        qos_profile = choose_qos(node, topic_name=topic, qos_args=qos_args)

        node.create_subscription(
            msg_class,
            topic,
            functools.partial(rt.callback, topic=topic),
            qos_profile,
            raw=True
        )
        print('Subscribed to [%s]' % topic)

    # remove the topics from the list if failed to find message type
    for topic in topics_to_be_removed:
        topics.remove(topic)
    if len(topics) == 0:
        node.destroy_node()
        rclpy.try_shutdown()
        return 1

    try:
        def thread_func():
            while rclpy.ok():
                rt.print_bw(topics, clear_screen=all_topics)
                time.sleep(1.0)

        print_thread = threading.Thread(target=thread_func)
        print_thread.start()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Suppress shutdown exceptions; cleanup is handled in finally block.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        print_thread.join()
    return 0
