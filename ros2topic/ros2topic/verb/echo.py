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

from collections import OrderedDict
import importlib
import sys

import rclpy
from rclpy.qos import qos_profile_sensor_data
from ros2cli.node.direct import DirectNode
from ros2topic.api import get_topic_names_and_types
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension
import yaml


class EchoVerb(VerbExtension):
    """Output messages from a topic."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to listen to (e.g. '/chatter')")
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')
        parser.add_argument(
            'message_type', nargs='?',
            help="Type of the ROS message (e.g. 'std_msgs/String')")
        parser.add_argument(
            '--csv', action='store_true',
            help='Output all recursive fields separated by commas (e.g. for '
                 'plotting)')

    def main(self, *, args):
        return main(args)


def main(args):
    if not args.csv:
        register_yaml_representer()
        callback = subscriber_cb
    else:
        callback = subscriber_cb_csv
    with DirectNode(args) as node:
        subscriber(node, args.topic_name, args.message_type, callback)


def register_yaml_representer():
    # Register our custom representer for YAML output
    yaml.add_representer(OrderedDict, represent_ordereddict)


# Custom representer for getting clean YAML output that preserves the order in
# an OrderedDict.
# Inspired by:
# http://stackoverflow.com/a/16782282/7169408
def represent_ordereddict(dumper, data):
    items = []
    for k, v in data.items():
        items.append((dumper.represent_data(k), dumper.represent_data(v)))
    return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', items)


def subscriber(node, topic_name, message_type, callback):
    if message_type is None:
        topic_names_and_types = get_topic_names_and_types(node=node)
        for n, t in topic_names_and_types:
            if n == topic_name:
                if len(t) > 1:
                    raise RuntimeError(
                        "Cannot echo topic '%s', as it contains more than one type: [%s]" %
                        (topic_name, ', '.join(t))
                    )
                message_type = t[0]
                break
        else:
            raise RuntimeError(
                'Could not determine the type for the passed topic')

    # TODO(dirk-thomas) this logic should come from a rosidl related package
    try:
        package_name, message_name = message_type.split('/', 2)
    except ValueError:
        raise RuntimeError('The passed message type is invalid')
    module = importlib.import_module(package_name + '.msg')
    msg_module = getattr(module, message_name)

    node.create_subscription(
        msg_module, topic_name, callback, qos_profile=qos_profile_sensor_data)

    while rclpy.ok():
        rclpy.spin_once(node)


def subscriber_cb(msg):
    print(msg_to_yaml(msg))


def msg_to_yaml(msg):
    return yaml.dump(msg_to_ordereddict(msg), width=sys.maxsize)


def subscriber_cb_csv(msg):
    print(msg_to_csv(msg))


def msg_to_csv(msg):
    def to_string(val):
        r = ''
        if any([isinstance(val, t) for t in [list, tuple]]):
            for v in val:
                if r:
                    r += ','
                r += to_string(v)
        elif any([isinstance(val, t) for t in [bool, bytes, float, int, str]]):
            r = str(val)
        else:
            r = msg_to_csv(val)
        return r
    result = ''
    # We rely on __slots__ retaining the order of the fields in the .msg file.
    for field_name in msg.__slots__:
        value = getattr(msg, field_name, None)
        if result:
            result += ','
        result += to_string(value)
    return result


# Convert a msg to an OrderedDict. We do this instead of implementing a generic
# __dict__() method in the msg because we want to preserve order of fields from
# the .msg file(s).
def msg_to_ordereddict(msg):
    d = OrderedDict()
    # We rely on __slots__ retaining the order of the fields in the .msg file.
    types = [bool, bytes, dict, float, int, list, str, tuple, OrderedDict]
    for field_name in msg.__slots__:
        value = getattr(msg, field_name, None)
        if not any([isinstance(value, t) for t in types]):
            value = msg_to_ordereddict(value)
        # remove leading underscore from field name
        d[field_name[1:]] = value
    return d
