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

from argparse import ArgumentParser
from argparse import ArgumentTypeError
from time import sleep
from typing import Optional

import warnings

from argcomplete import CompletionFinder

import rclpy

from rclpy.expand_topic_name import expand_topic_name
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from rclpy.validate_full_topic_name import validate_full_topic_name

from ros2cli.node.strategy import NodeStrategy

from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message


def positive_int(string):
    try:
        value = int(string)
    except ValueError:
        value = -1
    if value <= 0:
        raise ArgumentTypeError('value must be a positive integer')
    return value


def positive_float(inval):
    try:
        ret = float(inval)
    except ValueError:
        raise ArgumentTypeError('Expects a floating point number')
    if ret <= 0.0:
        raise ArgumentTypeError('Value must be positive')
    return ret


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

    try:
        return get_message(message_type)
    except (AttributeError, ModuleNotFoundError, ValueError):
        raise RuntimeError("The message type '%s' is invalid" % message_type)


class YamlCompletionFinder(CompletionFinder):
    def quote_completions(
        self, completions: list[str],
            cword_prequote: str, last_wordbreak_pos: Optional[int]):

        # For YAML content, return as-is without escaping
        if not any('-' in c for c in completions):
            return completions
        return super().quote_completions(completions, cword_prequote, last_wordbreak_pos)


class TopicMessagePrototypeCompleter:
    """Callable returning a message prototype."""

    def __init__(self, *, topic_type_key=None):
        self.topic_type_key = topic_type_key

    def __call__(self, prefix, parsed_args, **kwargs):
        message = get_message(getattr(parsed_args, self.topic_type_key))
        yaml_snippet = "'" + message_to_yaml(message()) + "'"
        return [yaml_snippet]


def profile_configure_short_keys(
    profile: rclpy.qos.QoSProfile = None, reliability: Optional[str] = None,
    durability: Optional[str] = None, depth: Optional[int] = None, history: Optional[str] = None,
    liveliness: Optional[str] = None, liveliness_lease_duration_s: Optional[int] = None,
) -> None:
    warnings.warn(
        "'qos_profile_from_short_keys()' is deprecated. "
        "use 'ros2cli.qos.qos_profile_from_short_keys()' instead.",
        DeprecationWarning)
    from ros2cli.qos import profile_configure_short_keys as _profile_configure_short_keys
    return _profile_configure_short_keys(profile, reliability, durability, depth, history,
                                         liveliness, liveliness_lease_duration_s)


def qos_profile_from_short_keys(
    preset_profile: str, reliability: Optional[str] = None, durability: Optional[str] = None,
    depth: Optional[int] = None, history: Optional[str] = None, liveliness: Optional[str] = None,
    liveliness_lease_duration_s: Optional[float] = None,
) -> rclpy.qos.QoSProfile:
    warnings.warn(
        "'qos_profile_from_short_keys()' is deprecated. "
        "use 'ros2cli.qos.qos_profile_from_short_keys()' instead.",
        DeprecationWarning)
    from ros2cli.qos import qos_profile_from_short_keys as _qos_profile_from_short_keys
    return _qos_profile_from_short_keys(
        preset_profile, reliability, durability,
        depth, history, liveliness, liveliness_lease_duration_s)


def add_qos_arguments(
    parser: ArgumentParser, entity_type: str,
    default_profile_str: str = 'default', extra_message: str = ''
) -> None:
    warnings.warn(
        "'add_qos_arguments()' is deprecated. "
        "use 'ros2cli.qos.add_qos_arguments()' instead.",
        DeprecationWarning)
    from ros2cli.qos import add_qos_arguments as _add_qos_arguments
    return _add_qos_arguments(parser, entity_type, default_profile_str, extra_message)


def choose_qos(node, topic_name: str, qos_args):
    warnings.warn(
        "'choose_qos()' is deprecated. "
        "use 'ros2cli.qos.choose_qos()' instead.",
        DeprecationWarning)
    from ros2cli.qos import choose_qos as _choose_qos
    return _choose_qos(node, topic_name, qos_args)
