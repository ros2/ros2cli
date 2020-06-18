# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from argparse import ArgumentTypeError
from time import sleep

import rclpy

from rclpy.expand_topic_name import expand_topic_name
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from rclpy.validate_full_topic_name import validate_full_topic_name
from ros2cli.node.strategy import NodeStrategy
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message


def unsigned_int(string):
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value < 0:
        raise ArgumentTypeError('value must be non-negative integer')
    return value


def get_topic_names_and_types(*, node, include_hidden_topics=False):
    topic_names_and_types = node.get_topic_names_and_types()
    if not include_hidden_topics:
        topic_names_and_types = [
            (n, t) for (n, t) in topic_names_and_types
            if not topic_or_service_is_hidden(n)]
    return topic_names_and_types


def get_topic_names(*, node, include_hidden_topics=False):
    topic_names_and_types = get_topic_names_and_types(
        node=node, include_hidden_topics=include_hidden_topics)
    return [n for (n, t) in topic_names_and_types]


class TopicNameCompleter:
    """Callable returning a list of topic names."""

    def __init__(self, *, include_hidden_topics_key=None):
        self.include_hidden_topics_key = include_hidden_topics_key

    def __call__(self, prefix, parsed_args, **kwargs):
        with NodeStrategy(parsed_args) as node:
            return get_topic_names(
                node=node,
                include_hidden_topics=getattr(
                    parsed_args, self.include_hidden_topics_key))


def message_type_completer(**kwargs):
    """Callable returning a list of message types."""
    message_types = []
    message_types_dict = get_message_interfaces()
    for package_name in sorted(message_types_dict.keys()):
        for message_name in sorted(message_types_dict[package_name]):
            message_types.append(f'{package_name}/{message_name}')
    return message_types


class TopicTypeCompleter:
    """Callable returning an existing topic type or all message types."""

    def __init__(self, *, topic_name_key=None):
        self.topic_name_key = topic_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        if self.topic_name_key is not None:
            with NodeStrategy(parsed_args) as node:
                topic_name = getattr(parsed_args, self.topic_name_key)
                names_and_types = get_topic_names_and_types(
                    node=node, include_hidden_topics=True)
                for n, t in names_and_types:
                    if n == topic_name:
                        return t
        return message_type_completer()


def get_msg_class(node, topic, blocking=False, include_hidden_topics=False):
    msg_class = _get_msg_class(node, topic, include_hidden_topics)
    if msg_class:
        return msg_class
    elif blocking:
        print('WARNING: topic [%s] does not appear to be published yet' % topic)
        while rclpy.ok():
            msg_class = _get_msg_class(node, topic, include_hidden_topics)
            if msg_class:
                return msg_class
            else:
                sleep(0.1)
    else:
        print('WARNING: topic [%s] does not appear to be published yet' % topic)
    return None


def _get_msg_class(node, topic, include_hidden_topics):
    """
    Get message module based on topic name.

    :param topic: topic name, ``list`` of ``str``
    """
    topic_names_and_types = get_topic_names_and_types(
        node=node, include_hidden_topics=include_hidden_topics)
    try:
        expanded_name = expand_topic_name(topic, node.get_name(), node.get_namespace())
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
                    (topic, ', '.join(t))
                )
            message_type = t[0]
            break
    else:
        # Could not determine the type for the passed topic
        return None

    return get_message(message_type)


class TopicMessagePrototypeCompleter:
    """Callable returning a message prototype."""

    def __init__(self, *, topic_type_key=None):
        self.topic_type_key = topic_type_key

    def __call__(self, prefix, parsed_args, **kwargs):
        message = get_message(getattr(parsed_args, self.topic_type_key))
        return [message_to_yaml(message())]


def qos_profile_from_short_keys(
    preset_profile: str, reliability: str = None, durability: str = None,
    depth: int = -1, history: str = None,
) -> rclpy.qos.QoSProfile:
    """Construct a QoSProfile given the name of a preset, and optional overrides."""
    # Build a QoS profile based on user-supplied arguments
    profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(preset_profile)
    if history:
        profile.history = rclpy.qos.QoSHistoryPolicy.get_from_short_key(history)
    if durability:
        profile.durability = rclpy.qos.QoSDurabilityPolicy.get_from_short_key(durability)
    if reliability:
        profile.reliability = rclpy.qos.QoSReliabilityPolicy.get_from_short_key(reliability)
    if depth >= 0:
        profile.depth = depth
    else:
        if (profile.durability == rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
                and profile.depth == 0):
            profile.depth = 1

    return profile


def add_qos_arguments_to_argument_parser(
    parser: argparse.ArgumentParser, is_publisher: bool = True, default_preset: str = 'sensor_data'
) -> None:
    """Extend an existing ArgumentParser to allow input of QoS policy overrides."""
    verb = 'publish' if is_publisher else 'subscribe'
    parser.add_argument(
        '--qos-profile',
        choices=rclpy.qos.QoSPresetProfiles.short_keys(),
        default=default_preset,
        help='Quality of service preset profile to {} with (default: {})'
             .format(verb, default_preset))
    default_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(
        default_preset)
    parser.add_argument(
        '--qos-depth', metavar='N', type=int, default=-1,
        help='Queue size setting to {} with '
             '(overrides depth value of --qos-profile option)'
             .format(verb))
    parser.add_argument(
        '--qos-history',
        choices=rclpy.qos.QoSHistoryPolicy.short_keys(),
        help='History of samples setting to {} with '
             '(overrides history value of --qos-profile option, default: {})'
             .format(verb, default_profile.history.short_key))
    parser.add_argument(
        '--qos-reliability',
        choices=rclpy.qos.QoSReliabilityPolicy.short_keys(),
        help='Quality of service reliability setting to {} with '
             '(overrides reliability value of --qos-profile option, default: {})'
             .format(verb, default_profile.reliability.short_key))
    parser.add_argument(
        '--qos-durability',
        choices=rclpy.qos.QoSDurabilityPolicy.short_keys(),
        help='Quality of service durability setting to {} with '
             '(overrides durability value of --qos-profile option, default: {})'
             .format(verb, default_profile.durability.short_key))
