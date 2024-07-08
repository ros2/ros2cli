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

import argparse
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.utilities import remove_ros_args
from rosidl_runtime_py.utilities import get_message


class RepeaterNode(Node):

    def __init__(self, message_type):
        super().__init__('repeater_node')
        self.message_type = message_type
        self.pub = self.create_publisher(
            self.message_type, '~/output', qos_profile_system_default
        )
        self.tmr = self.create_timer(1.0, self.callback)

    def callback(self):
        self.pub.publish(self.message_type())


def parse_arguments(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        'message_type', type=get_message,
        help='Message type for the repeater to publish.'
    )
    return parser.parse_args(args=remove_ros_args(args))


def main(args=None):
    try:
        parsed_args = parse_arguments(args=args)

        with rclpy.init(args=args):
            node = RepeaterNode(message_type=parsed_args.message_type)
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('repeater stopped cleanly')


if __name__ == '__main__':
    main(args=sys.argv[1:])
