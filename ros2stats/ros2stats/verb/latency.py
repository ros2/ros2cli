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

"""Implementation of 'ros2 latency' verb for end-to-end latency measurement."""

import sys
import time
import signal
import threading

from ros2cli.verb import VerbExtension

from ros2stats.api import (
    LatencyStats,
    LatencyMeasurement,
    format_latency_stats,
    format_latency_stats_json,
)


class LatencyVerb(VerbExtension):
    """Measure end-to-end latency between publishers and subscribers."""

    def add_arguments(self, parser, cli_name):
        """Add command-line arguments for the latency verb."""
        parser.add_argument(
            'topic',
            nargs='?',
            help='Topic to measure latency for (must have header with timestamp)'
        )
        parser.add_argument(
            '--samples', '-s',
            type=int,
            default=100,
            help='Number of samples to collect (default: 100)'
        )
        parser.add_argument(
            '--json', '-j',
            action='store_true',
            help='Output results in JSON format'
        )
        parser.add_argument(
            '--histogram', '-H',
            action='store_true',
            default=True,
            help='Show latency histogram (default: True)'
        )
        parser.add_argument(
            '--no-histogram',
            action='store_true',
            help='Hide latency histogram'
        )
        parser.add_argument(
            '--percentiles', '-p',
            action='store_true',
            default=True,
            help='Show percentile breakdown (default: True)'
        )
        parser.add_argument(
            '--timeout', '-t',
            type=float,
            default=30.0,
            help='Timeout in seconds (default: 30.0)'
        )
        parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            help='Show detailed information during collection'
        )

    def main(self, *, args):
        """Execute the latency verb."""
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import QoSProfile, ReliabilityPolicy
            from rosidl_runtime_py.utilities import get_message
        except ImportError as e:
            print(f'Error: Required ROS 2 packages not available: {e}', file=sys.stderr)
            return 1

        if not args.topic:
            print('Error: Please specify a topic name', file=sys.stderr)
            return 1

        rclpy.init()
        node = Node('_latency_measurement_node')

        # Find topic type
        topic_names_and_types = node.get_topic_names_and_types()
        topic_type = None
        for name, types in topic_names_and_types:
            if name == args.topic and types:
                topic_type = types[0]
                break

        if not topic_type:
            print(f'Error: Topic {args.topic} not found', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 1

        try:
            msg_class = get_message(topic_type)
        except Exception as e:
            print(f'Error: Could not load message type {topic_type}: {e}', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 1

        # Get publisher info for source name
        pub_info = node.get_publishers_info_by_topic(args.topic)
        source_name = pub_info[0].node_name if pub_info else 'unknown'

        # Create latency measurement
        measurement = LatencyMeasurement(args.samples)

        # Progress tracking
        samples_received = 0
        no_header_warning_shown = False

        def message_callback(msg):
            nonlocal samples_received, no_header_warning_shown

            recv_time = time.time()

            # Get header timestamp
            if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                stamp = msg.header.stamp
                header_time = stamp.sec + stamp.nanosec * 1e-9

                # Calculate latency
                latency_ms = (recv_time - header_time) * 1000

                # Ignore negative latencies (clock skew) and unreasonably high ones
                if 0 <= latency_ms < 10000:
                    measurement.record_sample(latency_ms)
                    samples_received = measurement.sample_count_current

                    if args.verbose:
                        print(f'\rCollected {samples_received}/{args.samples} samples '
                              f'(current: {latency_ms:.1f} ms)', end='', flush=True)
            else:
                if not no_header_warning_shown:
                    print(f'Warning: Message type {topic_type} does not have a header '
                          f'with timestamp. Cannot measure latency.', file=sys.stderr)
                    no_header_warning_shown = True

        # Create subscription with permissive QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        subscription = node.create_subscription(
            msg_class,
            args.topic,
            message_callback,
            qos
        )

        # Handle Ctrl+C
        running = True

        def signal_handler(sig, frame):
            nonlocal running
            running = False
            print('\nMeasurement interrupted.')

        signal.signal(signal.SIGINT, signal_handler)

        # Spin in background
        spin_thread = threading.Thread(
            target=lambda: self._spin_loop(rclpy, node, lambda: running)
        )
        spin_thread.start()

        # Wait for samples
        print(f'Measuring latency on {args.topic}...')
        if args.verbose:
            print(f'Collecting {args.samples} samples (timeout: {args.timeout}s)')

        start_time = time.time()
        while running:
            if measurement.sample_count_current >= args.samples:
                break
            if (time.time() - start_time) >= args.timeout:
                print(f'\nTimeout reached. Collected {measurement.sample_count_current} samples.')
                break
            time.sleep(0.1)

        running = False
        spin_thread.join(timeout=1.0)

        if args.verbose:
            print()  # New line after progress

        # Get and display results
        stats = measurement.get_stats(
            source=f'{source_name} ({args.topic})',
            destination='_latency_measurement_node'
        )

        if stats.samples == 0:
            print('Error: No valid latency samples collected.', file=sys.stderr)
            print('Make sure the message type has a header with timestamp.', file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 1

        if args.json:
            print(format_latency_stats_json(stats))
        else:
            show_histogram = args.histogram and not args.no_histogram
            print(format_latency_stats(stats, show_histogram=show_histogram))

        node.destroy_node()
        rclpy.shutdown()

        return 0

    def _spin_loop(self, rclpy, node, is_running):
        """Background spin loop."""
        while is_running():
            rclpy.spin_once(node, timeout_sec=0.1)
