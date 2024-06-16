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

from time import sleep
from typing import Optional

import rclpy

from argparse import ArgumentParser
from rclpy.expand_topic_name import expand_topic_name
from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from rclpy.validate_full_topic_name import validate_full_topic_name
from rosidl_runtime_py import get_message_interfaces
from rosidl_runtime_py import message_to_yaml
from rosidl_runtime_py.utilities import get_message


def profile_configure_short_keys(
    profile: rclpy.qos.QoSProfile = None, reliability: str = None,
    durability: str = None, depth: Optional[int] = None, history: str = None,
) -> rclpy.qos.QoSProfile:
    """Configure a QoSProfile given a profile, and optional overrides."""
    if history:
        profile.history = rclpy.qos.QoSHistoryPolicy.get_from_short_key(history)
    if durability:
        profile.durability = rclpy.qos.QoSDurabilityPolicy.get_from_short_key(durability)
    if reliability:
        profile.reliability = rclpy.qos.QoSReliabilityPolicy.get_from_short_key(reliability)
    if depth and depth >= 0:
        profile.depth = depth
    else:
        if (profile.durability == rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
                and profile.depth == 0):
            profile.depth = 1


def qos_profile_from_short_keys(
    preset_profile: str, reliability: str = None, durability: str = None,
    depth: Optional[int] = None, history: str = None,
) -> rclpy.qos.QoSProfile:
    """Construct a QoSProfile given the name of a preset, and optional overrides."""
    # Build a QoS profile based on user-supplied arguments
    profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(preset_profile)
    profile_configure_short_keys(profile, reliability, durability, depth, history)
    return profile


def add_qos_arguments(parser: ArgumentParser, default_profile_str: str):
    parser.add_argument(
        '--qos-profile',
        choices=rclpy.qos.QoSPresetProfiles.short_keys(),
        default=default_profile_str,
        help='Quality of service preset profile to subscribe with (default: {})'
             .format(default_profile_str))
    default_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(default_profile_str)
    parser.add_argument(
        '--qos-depth', metavar='N', type=int,
        help='Queue size setting to subscribe with '
             '(overrides depth value of --qos-profile option)')
    parser.add_argument(
        '--qos-history',
        choices=rclpy.qos.QoSHistoryPolicy.short_keys(),
        help='History of samples setting to subscribe with '
             '(overrides history value of --qos-profile option, default: {})'
             .format(default_profile.history.short_key))
    parser.add_argument(
        '--qos-reliability',
        choices=rclpy.qos.QoSReliabilityPolicy.short_keys(),
        help='Quality of service reliability setting to subscribe with '
             '(overrides reliability value of --qos-profile option, default: '
             'Automatically match existing publishers )')
    parser.add_argument(
        '--qos-durability',
        choices=rclpy.qos.QoSDurabilityPolicy.short_keys(),
        help='Quality of service durability setting to subscribe with '
             '(overrides durability value of --qos-profile option, default: '
             'Automatically match existing publishers )')
