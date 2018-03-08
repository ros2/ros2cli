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

import importlib
import time

import rclpy
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
            '-1', '--once', action='store_true',
            help='Publish one message and exit')
        parser.add_argument(
            '-n', '--node-name', type=str, default='',
            help='Name of the created publishing node')

    def main(self, *, args):
        if args.rate <= 0:
            raise RuntimeError('rate must be greater than zero')

        return main(args)


def main(args):
    return publisher(
        args.message_type, args.topic_name, args.values,
        args.node_name, 1. / args.rate, args.once)


def publisher(message_type, topic_name, values, node_name, period, once):
    # TODO(dirk-thomas) this logic should come from a rosidl related package
    try:
        package_name, message_name = message_type.split('/', 2)
        if not package_name or not message_name:
            raise ValueError()
    except ValueError:
        raise RuntimeError('The passed message type is invalid')
    module = importlib.import_module(package_name + '.msg')
    msg_module = getattr(module, message_name)
    values_dictionary = yaml.load(values)
    if not isinstance(values_dictionary, dict):
        return 'The passed value needs to be a dictionary in YAML format'
    if node_name == '':
        node_name = 'publisher_%s_%s' % (package_name, message_name)
    rclpy.init()

    node = rclpy.create_node(node_name)

    pub = node.create_publisher(msg_module, topic_name)

    msg = msg_module()
    try:
        set_msg_fields(msg, values_dictionary)
    except SetFieldError as e:
        return "Failed to populate field '{e.field_name}': {e.exception}" \
            .format_map(locals())

    print('publisher: beginning loop')
    while rclpy.ok():
        pub.publish(msg)
        print('publishing %r\n' % msg)
        if once:
            time.sleep(0.1)  # make sure the message reaches the wire before exiting
            break
        time.sleep(period)
    node.destroy_node()
    rclpy.shutdown()
