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

import hashlib
import os
import tempfile
import time
from typing import List, Optional, Tuple, TypeVar

from prompt_toolkit import prompt
from prompt_toolkit.formatted_text import HTML
from prompt_toolkit.key_binding import KeyBindings
from prompt_toolkit.lexers import PygmentsLexer
from pygments.lexers.data import YamlLexer
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from ros2cli.node.direct import add_arguments as add_direct_node_arguments
from ros2cli.node.direct import DirectNode
from ros2topic.api import profile_configure_short_keys
from ros2topic.api import TopicMessagePrototypeCompleter
from ros2topic.api import TopicNameCompleter
from ros2topic.api import TopicTypeCompleter
from ros2topic.verb import VerbExtension
from rosidl_runtime_py import set_message_fields
from rosidl_runtime_py.convert import message_to_yaml
from rosidl_runtime_py.utilities import get_message
import yaml

MsgType = TypeVar('MsgType')


def nonnegative_int(inval):
    ret = int(inval)
    if ret < 0:
        # The error message here gets completely swallowed by argparse
        raise ValueError('Value must be positive or zero')
    return ret


def positive_float(inval):
    ret = float(inval)
    if ret <= 0.0:
        # The error message here gets completely swallowed by argparse
        raise ValueError('Value must be positive')
    return ret


def get_pub_qos_profile():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=1)


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
        arg = parser.add_argument(
            'values', nargs='?', default='{}',
            help='Values to fill the message with in YAML format '
                 "(e.g. 'data: Hello World'), "
                 'otherwise the message will be published with default values')
        arg.completer = TopicMessagePrototypeCompleter(
            topic_type_key='message_type')
        parser.add_argument(
            '-r', '--rate', metavar='N', type=positive_float, default=1.0,
            help='Publishing rate in Hz (default: 1)')
        parser.add_argument(
            '-p', '--print', metavar='N', type=int, default=1,
            help='Only print every N-th published message (default: 1)')
        parser.add_argument(
            '-i', '--interactive', action='store_true',
            help='Interactively edit and send the message')
        group = parser.add_mutually_exclusive_group()
        group.add_argument(
            '-1', '--once', action='store_true',
            help='Publish one message and exit')
        group.add_argument(
            '-t', '--times', type=nonnegative_int, default=0,
            help='Publish this number of times and then exit')
        parser.add_argument(
            '-w', '--wait-matching-subscriptions', type=nonnegative_int, default=None,
            help=(
                'Wait until finding the specified number of matching subscriptions. '
                'Defaults to 1 when using "-1"/"--once"/"--times", otherwise defaults to 0.'))
        parser.add_argument(
            '--keep-alive', metavar='N', type=positive_float, default=0.1,
            help='Keep publishing node alive for N seconds after the last msg '
                 '(default: 0.1)')
        parser.add_argument(
            '-n', '--node-name',
            help='Name of the created publishing node')
        parser.add_argument(
            '--qos-profile',
            choices=rclpy.qos.QoSPresetProfiles.short_keys(),
            help='Quality of service preset profile to publish)')
        default_profile = get_pub_qos_profile()
        parser.add_argument(
            '--qos-depth', metavar='N', type=int, default=-1,
            help='Queue size setting to publish with '
                 '(overrides depth value of --qos-profile option)')
        parser.add_argument(
            '--qos-history',
            choices=rclpy.qos.QoSHistoryPolicy.short_keys(),
            help='History of samples setting to publish with '
                 '(overrides history value of --qos-profile option, default: {})'
                 .format(default_profile.history.short_key))
        parser.add_argument(
            '--qos-reliability',
            choices=rclpy.qos.QoSReliabilityPolicy.short_keys(),
            help='Quality of service reliability setting to publish with '
                 '(overrides reliability value of --qos-profile option, default: {})'
                 .format(default_profile.reliability.short_key))
        parser.add_argument(
            '--qos-durability',
            choices=rclpy.qos.QoSDurabilityPolicy.short_keys(),
            help='Quality of service durability setting to publish with '
                 '(overrides durability value of --qos-profile option, default: {})'
                 .format(default_profile.durability.short_key))
        add_direct_node_arguments(parser)

    def main(self, *, args):
        return main(args)


def main(args) -> Optional[str]:
    qos_profile = get_pub_qos_profile()

    qos_profile_name = args.qos_profile
    if qos_profile_name:
        qos_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(qos_profile_name)
    profile_configure_short_keys(
        qos_profile, args.qos_reliability, args.qos_durability,
        args.qos_depth, args.qos_history)

    times = args.times
    if args.once:
        times = 1

    if args.interactive:
        print('Interactive mode...')
        # Read last message that was sent if it exists in temp
        content = get_last_message_content(args.message_type)
        # Show the tui
        orig_msg, orig_timestamp_fields = parse_msg(args.message_type, content)
        default_msg, default_timestamp_fields = parse_msg(args.message_type)
        content = show_interactive_tui(message_to_yaml(orig_msg), message_to_yaml(default_msg))
        # Load msg YAML now to be sure it does not fail and we store a broken message
        msg, timestamp_fields = parse_msg(args.message_type, content)
        # Store the user input so we are able to load it the next time
        store_message_content(args.message_type, content)
    else:
        # Parse the yaml string and get a message object of the desired type
        msg, timestamp_fields = parse_msg(args.message_type, content)

    with DirectNode(args, node_name=args.node_name) as node:
        return publisher(
            node.node,
            args.topic_name,
            msg,
            timestamp_fields,
            1. / args.rate,
            args.print,
            times,
            args.wait_matching_subscriptions
            if args.wait_matching_subscriptions is not None else int(times != 0),
            qos_profile,
            args.keep_alive)


def get_history_file(msg_type: str) -> str:
    """
    Get paths for semi persistent history based on message name.

    :param msg_type: Name of the message type
    :returns: The path where a history file would be located if it exists
    """
    # Get temporary directory name
    msg_history_cache_folder_path = os.path.join(
        tempfile.gettempdir(), "ros_interactive_msg_cache")
    # Create temporary history dir if needed
    os.makedirs(msg_history_cache_folder_path, exist_ok=True)
    # Create a file based on the message name
    return os.path.join(
        msg_history_cache_folder_path,
        f'{hashlib.sha224(msg_type.encode()).hexdigest()[:20]}.yaml')


def get_last_message_content(msg_type: str) -> str:
    """
    Retrieve the last message of the given type that was sent using the tui if it exists.

    :param msg_type: Name of the message type
    :returns: The YAML representation containing the last message or an empty dict
    """
    content = "{}"
    try:
        history_path = get_history_file(msg_type)
        # Load previous values for that message type
        if os.path.exists(history_path):
            with open(history_path, 'r') as f:
                content = f.read()
    except OSError:
        print('Unable load history...')
    return content


def store_message_content(msg_type: str, content: str) -> None:
    """
    Store the YAML for the current message in a semi persistent file.

    :param msg_type: Name of the message type
    :param content: The YAML entered by the user
    """
    try:
        history_path = get_history_file(msg_type)
        # Clear cache
        if os.path.exists(history_path):
            os.remove(history_path)
        # Store last message in cache
        with open(history_path, 'w') as f:
            f.write(content)
    except OSError:
        print('Unable to store history')


def show_interactive_tui(msg_str: str, default_msg_str: Optional[str] = None) -> str:
    """
    Show a tui to edit a given message yaml.

    :param msg_str: Message yaml string which is initially presented to the user
    :param default_msg_str: Message yaml string with default values for the given message
    :return: The message yaml string edited by the user
    """
    # Create the bottom bar to pressent the options to the user
    def bottom_toolbar():
        return HTML(' Continue: <b>alt+enter</b> | Exit: <b>ctrl+c</b> | Reset: <b>ctrl+r</b>')

    # Create key bindings for the prompt
    bindings = KeyBindings()
    if default_msg_str is not None:
        @bindings.add('c-r')
        def _(event):
            """Reset the promt to the default message."""
            event.app.current_buffer.text = default_msg_str

    # Show prompt to edit the message before sending it
    return prompt(
        "> ",
        multiline=True,
        default=msg_str,
        lexer=PygmentsLexer(YamlLexer),
        mouse_support=True,
        bottom_toolbar=bottom_toolbar,
        key_bindings=bindings)


def parse_msg(msg_type: str, yaml_values: Optional[str] = None) -> Tuple[MsgType, List]:
    """
    Parse the name and contents of a given message.

    :param msg_type: Name of the message as a string (e.g. std_msgs/msg/Header)
    :param yaml_values: Contents of the message as a string in YAML layout
    :returns: An constructed instance of the message type
    """
    # Get the message type from the name string
    try:
        msg_module = get_message(msg_type)
    except (AttributeError, ModuleNotFoundError, ValueError):
        raise RuntimeError('The passed message type is invalid')
    # Create a default instance of the message with the given name
    msg = msg_module()
    timestamp_fields = []
    # Check if we want to add values to the message
    if yaml_values is not None:
        # Load the user provided fields of the message
        values_dictionary = yaml.safe_load(yaml_values)
        if not isinstance(values_dictionary, dict):
            raise RuntimeError('The passed value needs to be a dictionary in YAML format')
        # Set all fields in the message to the provided values
        try:
            timestamp_fields = set_message_fields(
                msg, values_dictionary, expand_header_auto=True, expand_time_now=True)
        except Exception as e:
            raise RuntimeError('Failed to populate field: {0}'.format(e))
    return msg, timestamp_fields


def publisher(
    node: Node,
    topic_name: str,
    msg: MsgType,
    timestamp_fields: list,
    period: float,
    print_nth: int,
    times: int,
    wait_matching_subscriptions: int,
    qos_profile: QoSProfile,
    keep_alive: float,
) -> None:
    """
    Initialize a node with a single publisher and run its publish loop (maybe only once).

    :param node: The node used for publishing the given message
    :param topic_name: The topic on which the the message is published
    :param msg: The message to be published
    :param timestamp_fields: Any timestamp fields that need to be populated
    :param period: Period after which the msg is published again
    :param print_nth: Interval in which the message is printed
    :param times: Number of times the message is published
    :param wait_matching_subscriptions: Wait until there is a certain number of subscribtions
    :param qos_profile: QoS profile
    :param keep_alive: Time the node is kept alive after the message was sent
    """
    pub = node.create_publisher(type(msg), topic_name, qos_profile)

    times_since_last_log = 0
    while pub.get_subscription_count() < wait_matching_subscriptions:
        # Print a message reporting we're waiting 1s, but check the condition every 100ms.
        if not times_since_last_log:
            print(
                f'Waiting for at least {wait_matching_subscriptions} matching subscription(s)...')
        times_since_last_log = (times_since_last_log + 1) % 10
        time.sleep(0.1)

    print('publisher: beginning loop')
    count = 0

    def timer_callback():
        stamp_now = node.get_clock().now().to_msg()
        for field_setter in timestamp_fields:
            field_setter(stamp_now)
        nonlocal count
        count += 1
        if print_nth and count % print_nth == 0:
            print('publishing #%d: %r\n' % (count, msg))
        pub.publish(msg)

    timer_callback()
    if times != 1:
        timer = node.create_timer(period, timer_callback)
        while times == 0 or count < times:
            rclpy.spin_once(node)
        node.destroy_timer(timer)

    # give some time for the messages to reach the wire before exiting
    time.sleep(keep_alive)
