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

from argparse import ArgumentTypeError
from collections import OrderedDict
import importlib
import sys

import rclpy
from rclpy.expand_topic_name import expand_topic_name
from rclpy.qos import qos_profile_sensor_data
from rclpy.validate_full_topic_name import validate_full_topic_name
from ros2cli.node.direct import DirectNode
from ros2topic.api import get_topic_names_and_types
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension
import yaml

DEFAULT_TRUNCATE_LENGTH = 128


def unsigned_int(string):
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value < 0:
        raise ArgumentTypeError('value must be non-negative integer')
    return value


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
        parser.add_argument(
            '--full-length', '-f', action='store_true',
            help='Output all elements for arrays, bytes, and string with a '
                 "length > '--truncate-length', by default they are truncated "
                 "after '--truncate-length' elements with '...''")
        parser.add_argument(
            '--truncate-length', '-l', type=unsigned_int, default=DEFAULT_TRUNCATE_LENGTH,
            help='The length to truncate arrays, bytes, and string to '
                 '(default: %d)' % DEFAULT_TRUNCATE_LENGTH)

    def main(self, *, args):
        return main(args)


def main(args):
    if not args.csv:
        register_yaml_representer()
        callback = subscriber_cb(args)
    else:
        callback = subscriber_cb_csv(args)
    with DirectNode(args) as node:
        subscriber(
            node.node, args.topic_name, args.message_type, callback)


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
        topic_names_and_types = get_topic_names_and_types(node=node, include_hidden_topics=True)
        try:
            expanded_name = expand_topic_name(topic_name, node.get_name(), node.get_namespace())
        except ValueError as e:
            raise RuntimeError(e)
        try:
            validate_full_topic_name(expanded_name)
        except rclpy.exceptions.InvalidTopicNameException as e:
            raise RuntimeError(e)
        for n, t in topic_names_and_types:
            if n == expanded_name:
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
        if not package_name or not message_name:
            raise ValueError()
    except ValueError:
        raise RuntimeError('The passed message type is invalid')
    module = importlib.import_module(package_name + '.msg')
    msg_module = getattr(module, message_name)

    node.create_subscription(
        msg_module, topic_name, callback, qos_profile=qos_profile_sensor_data)

    while rclpy.ok():
        rclpy.spin_once(node)


def subscriber_cb(args):
    def cb(msg):
        nonlocal args
        print(msg_to_yaml(args, msg))
    return cb


def msg_to_yaml(args, msg):
    return yaml.dump(
        msg_to_ordereddict(
            msg,
            truncate_length=args.truncate_length if not args.full_length else None
        ), width=sys.maxsize)


def subscriber_cb_csv(args):
    def cb(msg):
        nonlocal args
        print(msg_to_csv(args, msg))
    return cb


def msg_to_csv(args, msg):
    def to_string(val):
        nonlocal args
        r = ''
        if any(isinstance(val, t) for t in [list, tuple]):
            for i, v in enumerate(val):
                if r:
                    r += ','
                if not args.full_length and i >= args.truncate_length:
                    r += '...'
                    break
                r += to_string(v)
        elif any(isinstance(val, t) for t in [bool, bytes, float, int, str]):
            if any(isinstance(val, t) for t in [bytes, str]):
                if not args.full_length and len(val) > args.truncate_length:
                    val = val[:args.truncate_length]
                    if isinstance(val, bytes):
                        val += b'...'
                    else:
                        val += '...'
            r = str(val)
        else:
            r = msg_to_csv(args, val)
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
def msg_to_ordereddict(msg, truncate_length=None):
    d = OrderedDict()
    # We rely on __slots__ retaining the order of the fields in the .msg file.
    for field_name in msg.__slots__:
        value = getattr(msg, field_name, None)
        value = _convert_value(value, truncate_length=truncate_length)
        # remove leading underscore from field name
        d[field_name[1:]] = value
    return d


def _convert_value(value, truncate_length=None):
    if isinstance(value, bytes):
        if truncate_length is not None and len(value) > truncate_length:
            value = ''.join([chr(c) for c in value[:truncate_length]]) + '...'
        else:
            value = ''.join([chr(c) for c in value])
    elif isinstance(value, str):
        if truncate_length is not None and len(value) > truncate_length:
            value = value[:truncate_length] + '...'
    elif isinstance(value, tuple) or isinstance(value, list):
        if truncate_length is not None and len(value) > truncate_length:
                # Truncate the sequence
                value = value[:truncate_length]
                # Truncate every item in the sequence
                value = type(value)([_convert_value(v, truncate_length) for v in value] + ['...'])
        else:
            # Truncate every item in the list
            value = type(value)([_convert_value(v, truncate_length) for v in value])
    elif isinstance(value, dict) or isinstance(value, OrderedDict):
        # convert each key and value in the mapping
        new_value = {} if isinstance(value, dict) else OrderedDict()
        for k, v in value.items():
            # don't truncate keys because that could result in key collisions and data loss
            new_value[_convert_value(k)] = _convert_value(v, truncate_length=truncate_length)
        value = new_value
    elif not any(isinstance(value, t) for t in (bool, float, int)):
        # assuming value is a message
        # since it is neither a collection nor a primitive type
        value = msg_to_ordereddict(value, truncate_length=truncate_length)
    return value
