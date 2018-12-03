# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

import time

import rclpy
from ros2topic.api import import_message_type
from ros2topic.api import set_msg_fields
from ros2topic.api import SetFieldError
from ros2topic.api import TopicNameCompleter
from ros2topic.api import TopicTypeCompleter
from ros2topic.verb import VerbExtension
import yaml


class PubVerb(VerbExtension):
    """Publish a message to a topic."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to publish to (e.g. '/chatter')")
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')
        arg = parser.add_argument(
            'message_type',
            help="Type of the ROS message (e.g. 'std_msgs/String')")
        arg.completer = TopicTypeCompleter(
            topic_name_key='topic_name')
        parser.add_argument(
            'values', nargs='?', default='{}',
            help='Values to fill the message with in YAML format ' +
                 '(e.g. "data: Hello World"), ' +
                 'otherwise the message will be published with default values')
        parser.add_argument(
            '-r', '--rate', metavar='N', type=float, default=1.0,
            help='Publishing rate in Hz (default: 1)')
        parser.add_argument(
            '-p', '--print', metavar='N', type=int, default=1,
            help='Only print every N-th published message (default: 1)')
        parser.add_argument(
            '-1', '--once', action='store_true',
            help='Publish one message and exit')
        parser.add_argument(
            '-n', '--node-name', type=str,
            help='Name of the created publishing node')

    def main(self, *, args):
        if args.rate <= 0:
            raise RuntimeError('rate must be greater than zero')

        return main(args)


def main(args):
    return publisher(
        args.message_type, args.topic_name, args.values,
        args.node_name, 1. / args.rate, args.print, args.once)


def publisher(
    message_type, topic_name, values, node_name, period, print_nth, once
):
    msg_module = import_message_type(topic_name, message_type)
    values_dictionary = yaml.load(values)
    if not isinstance(values_dictionary, dict):
        return 'The passed value needs to be a dictionary in YAML format'
    if not node_name:
        node_name = 'publisher_%s' % (message_type.replace('/', '_'),)
    rclpy.init()

    node = rclpy.create_node(node_name)

    pub = node.create_publisher(msg_module, topic_name)

    msg = msg_module()
    try:
        set_msg_fields(msg, values_dictionary)
    except SetFieldError as e:  # noqa: F841
        return "Failed to populate field '{e.field_name}': {e.exception}" \
            .format_map(locals())

    print('publisher: beginning loop')
    count = 0

    def timer_callback():
        nonlocal count
        count += 1
        if print_nth and count % print_nth == 0:
            print('publishing #%d: %r\n' % (count, msg))
        pub.publish(msg)

    timer = node.create_timer(period, timer_callback)
    if once:
        rclpy.spin_once(node)
        time.sleep(0.1)  # make sure the message reaches the wire before exiting
    else:
        rclpy.spin(node)

    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()
