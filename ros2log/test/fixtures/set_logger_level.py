#!/usr/bin/env python3
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

"""Test helper that sets a node logger level from a fresh ROS process."""

import sys

from rcl_interfaces.msg import LoggerLevel
from rcl_interfaces.srv import SetLoggerLevels
import rclpy

from ros2log.api import get_logger_name_for_node
from ros2node.api import get_absolute_node_name


LEVEL_NAME_TO_VALUE = {
    'UNSET': LoggerLevel.LOG_LEVEL_UNKNOWN,
    'DEBUG': LoggerLevel.LOG_LEVEL_DEBUG,
    'INFO': LoggerLevel.LOG_LEVEL_INFO,
    'WARN': LoggerLevel.LOG_LEVEL_WARN,
    'ERROR': LoggerLevel.LOG_LEVEL_ERROR,
    'FATAL': LoggerLevel.LOG_LEVEL_FATAL,
}


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    if len(argv) != 2:
        print('usage: set_logger_level.py NODE_NAME LEVEL', file=sys.stderr)
        return 2

    node_name, level_name = argv
    if level_name not in LEVEL_NAME_TO_VALUE:
        print(f'unknown level: {level_name}', file=sys.stderr)
        return 2

    absolute_node_name = get_absolute_node_name(node_name)
    logger_level = LoggerLevel()
    logger_level.name = get_logger_name_for_node(absolute_node_name)
    logger_level.level = LEVEL_NAME_TO_VALUE[level_name]

    rclpy.init(args=[])
    node = rclpy.create_node('set_logger_level_helper')
    try:
        client = node.create_client(
            SetLoggerLevels,
            f'{absolute_node_name}/set_logger_levels',
        )
        if not client.wait_for_service(timeout_sec=10.0):
            print(
                'Wait for service timed out waiting for logger services '
                f'for node {absolute_node_name}',
                file=sys.stderr,
            )
            return 1

        request = SetLoggerLevels.Request()
        request.levels = [logger_level]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        response = future.result()
        if response is None:
            print(future.exception(), file=sys.stderr)
            return 1
        if len(response.results) != 1:
            print(
                'Unexpected response while setting logger level for node '
                f"'{absolute_node_name}'",
                file=sys.stderr,
            )
            return 1

        result = response.results[0]
        if not result.successful:
            message = 'Setting logger level failed'
            if result.reason:
                message += f': {result.reason}'
            print(message, file=sys.stderr)
            return 1

        return 0
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except RuntimeError as e:
            # A shutdown failure (e.g. rmw_zenoh session close timeout) must
            # not mask the result of the logger-level operation.
            print(f'Ignoring error during shutdown: {e}', file=sys.stderr)


if __name__ == '__main__':
    sys.exit(main())
