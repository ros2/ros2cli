# Copyright 2026 Tomoya Fujita, Fumiya Ohnishi
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

import re
import sys
from typing import Optional

from rcl_interfaces.msg import Log

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_rosout_default
from rclpy.qos import QoSProfile

from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode
from ros2cli.qos import add_qos_arguments
from ros2cli.qos import choose_qos

from ros2log.verb import VerbExtension


# Log level mapping
LOG_LEVELS = {
    'DEBUG': Log.DEBUG,
    'INFO': Log.INFO,
    'WARN': Log.WARN,
    'ERROR': Log.ERROR,
    'FATAL': Log.FATAL,
}

LOG_LEVEL_NAMES = {
    Log.DEBUG: 'DEBUG',
    Log.INFO: 'INFO',
    Log.WARN: 'WARN',
    Log.ERROR: 'ERROR',
    Log.FATAL: 'FATAL',
}

# ANSI color codes
COLOR_RESET = '\033[0m'
COLOR_DEBUG = '\033[37m'      # White
COLOR_INFO = '\033[32m'       # Green
COLOR_WARN = '\033[33m'       # Yellow
COLOR_ERROR = '\033[31m'      # Red
COLOR_FATAL = '\033[35m'      # Magenta

LOG_LEVEL_COLORS = {
    Log.DEBUG: COLOR_DEBUG,
    Log.INFO: COLOR_INFO,
    Log.WARN: COLOR_WARN,
    Log.ERROR: COLOR_ERROR,
    Log.FATAL: COLOR_FATAL,
}


class WatchVerb(VerbExtension):
    """Monitor and display logs in real-time."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--level',
            type=str,
            choices=['DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL'],
            help='Show only logs at or above the specified severity level')
        parser.add_argument(
            '--logger',
            type=str,
            help='Filter logs by logger name')
        parser.add_argument(
            '--regex',
            type=str,
            help='Filter log messages matching the specified regular expression pattern. '
                 'e.g. "topic.*(/\\w+)"')
        parser.add_argument(
            '--no-color',
            action='store_true',
            default=False,
            help='Disable colorized output')
        parser.add_argument(
            '--no-timestamp',
            action='store_true',
            default=False,
            help='Disable timestamp display')
        parser.add_argument(
            '--function-detail',
            action='store_true',
            default=False,
            help='Output function name, file, and line number')
        add_direct_node_arguments(parser)
        add_qos_arguments(
            parser,
            entity_type='subscribe',
            default_profile_str='rosout_default')

    def main(self, *, args):
        with DirectNode(args) as node:
            # Configure QoS profile based on arguments and available publishers
            qos_profile = choose_qos(node, '/rosout', args)
            try:
                LogWatcher(
                    node,
                    level_filter=args.level,
                    logger_filter=args.logger,
                    regex_filter=args.regex,
                    enable_color=not args.no_color,
                    show_timestamp=not args.no_timestamp,
                    show_function_detail=args.function_detail,
                    qos_profile=qos_profile,
                    debug=args.debug,
                )
            except ValueError as e:
                print(str(e), file=sys.stderr)
                return 1

            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass

        return 0


class LogWatcher:
    """Helper class to watch and filter logs from /rosout topic."""

    def __init__(
        self,
        node: Node,
        level_filter: Optional[str] = None,
        logger_filter: Optional[str] = None,
        regex_filter: Optional[str] = None,
        enable_color: bool = True,
        show_timestamp: bool = True,
        show_function_detail: bool = False,
        qos_profile: QoSProfile = qos_profile_rosout_default,
        enable_content_filter: bool = True,
        debug: bool = False,
    ) -> None:
        self.node = node
        self.enable_color = enable_color
        self.show_timestamp = show_timestamp
        self.show_function_detail = show_function_detail
        self.debug = debug

        # Set up level filter
        self.min_level = LOG_LEVELS.get(level_filter, Log.DEBUG) if level_filter else Log.DEBUG

        # Set up logger filter
        self.logger_filter = logger_filter

        # Set up regex filter
        self.regex_pattern = None
        if regex_filter:
            try:
                self.regex_pattern = re.compile(regex_filter)
            except re.error as e:
                raise ValueError(f'Invalid regex pattern: {e}') from e

        # Create subscription to /rosout
        self.subscription = node.create_subscription(
            Log,
            '/rosout',
            self._log_callback,
            qos_profile
        )

        # Try to set content filter for performance optimization
        # This filters at RMW level if supported, reducing network traffic and CPU usage
        if enable_content_filter:
            self._setup_content_filter()

    def _setup_content_filter(self):
        """
        Set up content filter for log level and logger name if supported by RMW.

        Content filtering is a DDS feature that filters messages at the middleware level,
        reducing network traffic and CPU usage. Currently, only DDS-based RMW implementations
        (e.g., FastDDS, Connext DDS) support this feature. DDS specifications limit the number
        of expression parameters to 100 at a time. If content filtering is not supported or
        fails, the implementation automatically falls back to client-side filtering.
        """
        filter_expressions = []
        expression_parameters = []

        # Add level filter expression
        if self.min_level > Log.DEBUG:
            filter_expressions.append('level >= %0')
            expression_parameters.append(str(self.min_level))

        # Add logger name filter expression
        if self.logger_filter:
            param_index = len(expression_parameters)
            filter_expressions.append(f'name = %{param_index}')
            # String parameters need to be quoted for DDS SQL filter
            expression_parameters.append(f"'{self.logger_filter}'")

        # Only set content filter if we have filter expressions
        if filter_expressions:
            filter_expression = ' AND '.join(filter_expressions)
            try:
                self.subscription.set_content_filter(filter_expression, expression_parameters)
                # Check if content filtering was successfully enabled
                if self.subscription.is_cft_enabled:
                    if self.debug:
                        print(
                            f'Content filter enabled: {filter_expression} with parameters '
                            f'{expression_parameters}')
                else:
                    if self.debug:
                        print(
                            'Content filtering not supported by RMW implementation. '
                            'Falling back to client-side filtering.')
            except Exception as ex:
                # Content filtering may not be supported by the RMW implementation
                if self.debug:
                    print(
                        f'Failed to set content filter: {ex}. '
                        'Falling back to client-side filtering.')

    def _log_callback(self, msg: Log):
        """Process log messages."""
        # Apply level filter
        if msg.level < self.min_level:
            return

        # Apply logger filter
        if self.logger_filter and msg.name != self.logger_filter:
            return

        # Apply regex filter on message text
        if self.regex_pattern and not self.regex_pattern.search(msg.msg):
            return

        # Format and print the log message
        self._print_log(msg)

    def _print_log(self, msg: Log):
        """Format and print a log message."""
        parts = []

        # Add timestamp if enabled
        if self.show_timestamp:
            timestamp_sec = msg.stamp.sec + msg.stamp.nanosec / 1e9
            parts.append(f'[{timestamp_sec:.6f}]')

        # Add log level with color
        level_name = LOG_LEVEL_NAMES.get(msg.level, 'UNKNOWN')
        if self.enable_color:
            color = LOG_LEVEL_COLORS.get(msg.level, COLOR_RESET)
            parts.append(f'{color}[{level_name}]{COLOR_RESET}')
        else:
            parts.append(f'[{level_name}]')

        # Add logger name
        parts.append(f'[{msg.name}]')

        # Add function details if enabled
        if self.show_function_detail:
            parts.append(f'[{msg.function}@{msg.file}:{msg.line}]')

        # Add message
        parts.append(f': {msg.msg}')

        # Print the formatted message
        print(' '.join(parts), flush=True)
