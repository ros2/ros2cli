# Copyright 2024 Open Source Robotics Foundation, Inc.
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

"""Implementation of 'ros2 topic stats' verb for topic statistics."""

import sys
import time
import signal
import threading
from collections import deque

from ros2cli.verb import VerbExtension

from ros2stats.api import (
    TopicStats,
    TopicStatisticsCollector,
    format_topic_stats,
    format_topic_stats_json,
    format_topic_stats_csv,
)


class StatsVerb(VerbExtension):
    """Display comprehensive statistics for ROS 2 topics."""

    def add_arguments(self, parser, cli_name):
        """Add command-line arguments for the stats verb."""
        parser.add_argument(
            'topic_name',
            nargs='?',
            help='Name of the topic to monitor'
        )
        parser.add_argument(
            '--all', '-a',
            action='store_true',
            help='Show stats for all topics'
        )
        parser.add_argument(
            '--window', '-w',
            type=int,
            default=100,
            help='Sample window size for rate calculations (default: 100)'
        )
        parser.add_argument(
            '--json', '-j',
            action='store_true',
            help='Output in JSON format'
        )
        parser.add_argument(
            '--csv', '-c',
            action='store_true',
            help='Output in CSV format'
        )
        parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            help='Show detailed information'
        )
        parser.add_argument(
            '--continuous',
            action='store_true',
            help='Continuous update mode'
        )
        parser.add_argument(
            '--interval', '-i',
            type=float,
            default=1.0,
            help='Update interval in seconds (default: 1.0)'
        )
        parser.add_argument(
            '--duration', '-d',
            type=float,
            default=0,
            help='Duration to collect stats (0 = until Ctrl+C)'
        )

    def main(self, *, args):
        """Execute the stats verb."""
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
            from rosidl_runtime_py.utilities import get_message
        except ImportError as e:
            print(f'Error: Required ROS 2 packages not available: {e}', file=sys.stderr)
            return 1

        if not args.topic_name and not args.all:
            print('Error: Please specify a topic name or use --all', file=sys.stderr)
            return 1

        rclpy.init()
        node = Node('_topic_stats_node')

        # Get list of topics to monitor
        topic_names_and_types = node.get_topic_names_and_types()

        if args.all:
            topics_to_monitor = [
                (name, types[0]) for name, types in topic_names_and_types
                if types and not name.startswith('/rosout') and not name.startswith('/parameter')
            ]
        else:
            # Find the requested topic
            topic_type = None
            for name, types in topic_names_and_types:
                if name == args.topic_name and types:
                    topic_type = types[0]
                    break

            if not topic_type:
                print(f'Error: Topic {args.topic_name} not found or has no type', file=sys.stderr)
                node.destroy_node()
                rclpy.shutdown()
                return 1

            topics_to_monitor = [(args.topic_name, topic_type)]

        # Create collectors and subscriptions
        collectors = {}
        subscriptions = []

        for topic_name, topic_type in topics_to_monitor:
            try:
                msg_class = get_message(topic_type)
            except Exception as e:
                print(f'Warning: Could not load message type {topic_type}: {e}', file=sys.stderr)
                continue

            collector = TopicStatisticsCollector(topic_name, args.window)
            collector._stats.message_type = topic_type

            # Get connection info
            pub_info = node.get_publishers_info_by_topic(topic_name)
            sub_info = node.get_subscriptions_info_by_topic(topic_name)

            collector._stats.publisher_count = len(pub_info)
            collector._stats.subscriber_count = len(sub_info) + 1  # +1 for our subscription
            collector._stats.publishers = [
                getattr(p, 'node_name', 'unknown') for p in pub_info
            ]

            collectors[topic_name] = collector

            def make_callback(coll):
                def callback(msg):
                    recv_time = time.time()
                    try:
                        size = sys.getsizeof(msg)
                    except Exception:
                        size = 0

                    header_stamp = None
                    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                        stamp = msg.header.stamp
                        header_stamp = stamp.sec + stamp.nanosec * 1e-9

                    coll.record_message(recv_time, size, header_stamp)
                return callback

            # Use a permissive QoS profile
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                depth=10
            )

            sub = node.create_subscription(
                msg_class,
                topic_name,
                make_callback(collector),
                qos
            )
            subscriptions.append(sub)

        if not collectors:
            print('Error: No valid topics to monitor', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 1

        # Handle Ctrl+C
        running = True

        def signal_handler(sig, frame):
            nonlocal running
            running = False

        signal.signal(signal.SIGINT, signal_handler)

        # Spin in background thread
        spin_thread = threading.Thread(target=lambda: self._spin_loop(rclpy, node, lambda: running))
        spin_thread.start()

        # Main display loop
        start_time = time.time()
        try:
            while running:
                # Clear screen in continuous mode
                if args.continuous:
                    print('\033[2J\033[H', end='')

                # Display stats for each topic
                for topic_name, collector in collectors.items():
                    stats = collector.get_stats()

                    if args.json:
                        print(format_topic_stats_json(stats))
                    elif args.csv:
                        print(format_topic_stats_csv(stats))
                    else:
                        print(format_topic_stats(stats, verbose=args.verbose))

                # Check duration
                if args.duration > 0 and (time.time() - start_time) >= args.duration:
                    break

                # Exit if not continuous
                if not args.continuous:
                    # Wait a bit for initial data
                    time.sleep(args.interval)
                    # Print final stats
                    if not args.json and not args.csv:
                        print('\033[2J\033[H', end='')
                    for topic_name, collector in collectors.items():
                        stats = collector.get_stats()
                        if args.json:
                            print(format_topic_stats_json(stats))
                        elif args.csv:
                            print(format_topic_stats_csv(stats))
                        else:
                            print(format_topic_stats(stats, verbose=args.verbose))
                    break

                time.sleep(args.interval)

        except KeyboardInterrupt:
            pass
        finally:
            running = False
            spin_thread.join(timeout=1.0)

            node.destroy_node()
            rclpy.shutdown()

        return 0

    def _spin_loop(self, rclpy, node, is_running):
        """Background spin loop."""
        while is_running():
            rclpy.spin_once(node, timeout_sec=0.1)
