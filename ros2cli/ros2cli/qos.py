# Copyright 2025 Sony Group Corporation.
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
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSReliabilityPolicy


def profile_configure_short_keys(
    profile: rclpy.qos.QoSProfile = None, reliability: Optional[str] = None,
    durability: Optional[str] = None, depth: Optional[int] = None, history: Optional[str] = None,
    liveliness: Optional[str] = None, liveliness_lease_duration_s: Optional[int] = None,
) -> None:
    """Configure a QoSProfile given a profile, and optional overrides."""
    if history:
        profile.history = rclpy.qos.QoSHistoryPolicy.get_from_short_key(history)
    if durability:
        profile.durability = rclpy.qos.QoSDurabilityPolicy.get_from_short_key(durability)
    if reliability:
        profile.reliability = rclpy.qos.QoSReliabilityPolicy.get_from_short_key(reliability)
    if liveliness:
        profile.liveliness = rclpy.qos.QoSLivelinessPolicy.get_from_short_key(liveliness)
    if liveliness_lease_duration_s and liveliness_lease_duration_s >= 0:
        profile.liveliness_lease_duration = Duration(seconds=liveliness_lease_duration_s)
    if depth and depth >= 0:
        profile.depth = depth
    else:
        if (profile.durability == rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
                and profile.depth == 0):
            profile.depth = 1


def qos_profile_from_short_keys(
    preset_profile: str, reliability: Optional[str] = None, durability: Optional[str] = None,
    depth: Optional[int] = None, history: Optional[str] = None, liveliness: Optional[str] = None,
    liveliness_lease_duration_s: Optional[float] = None,
) -> rclpy.qos.QoSProfile:
    """Construct a QoSProfile given the name of a preset, and optional overrides."""
    # Build a QoS profile based on user-supplied arguments
    profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(preset_profile)
    profile_configure_short_keys(
        profile, reliability, durability, depth, history, liveliness, liveliness_lease_duration_s)
    return profile


def add_qos_arguments(
    parser: ArgumentParser, entity_type: str,
    default_profile_str: str = 'default', extra_message: str = ''
) -> None:
    parser.add_argument(
        '--qos-profile',
        choices=rclpy.qos.QoSPresetProfiles.short_keys(),
        help=(
            f'Quality of service preset profile to {entity_type} with'
            f' (default: {default_profile_str})'),
        default=default_profile_str)
    default_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(default_profile_str)
    parser.add_argument(
        '--qos-depth', metavar='N', type=int,
        help=(
            f'Queue size setting to {entity_type} with '
            '(overrides depth value of --qos-profile option, default: '
            f'{default_profile.depth})'))
    parser.add_argument(
        '--qos-history',
        choices=rclpy.qos.QoSHistoryPolicy.short_keys(),
        help=(
            f'History of samples setting to {entity_type} with '
            '(overrides history value of --qos-profile option, default: '
            f'{default_profile.history.short_key})'))
    parser.add_argument(
        '--qos-reliability',
        choices=rclpy.qos.QoSReliabilityPolicy.short_keys(),
        help=(
            f'Quality of service reliability setting to {entity_type} with '
            '(overrides reliability value of --qos-profile option, default: '
            f'{default_profile.reliability.short_key} {extra_message})'))
    parser.add_argument(
        '--qos-durability',
        choices=rclpy.qos.QoSDurabilityPolicy.short_keys(),
        help=(
            f'Quality of service durability setting to {entity_type} with '
            '(overrides durability value of --qos-profile option, default: '
            f'{default_profile.durability.short_key}  {extra_message})'))
    parser.add_argument(
        '--qos-liveliness',
        choices=rclpy.qos.QoSLivelinessPolicy.short_keys(),
        help=(
            f'Quality of service liveliness setting to {entity_type} with '
            '(overrides liveliness value of --qos-profile option, default '
            f'{default_profile.liveliness.short_key})'))
    parser.add_argument(
        '--qos-liveliness-lease-duration-seconds',
        type=float,
        help=(
            f'Quality of service liveliness lease duration setting to {entity_type} '
            'with (overrides liveliness lease duration value of --qos-profile option, default: '
            f'{default_profile.liveliness_lease_duration})'))


def choose_qos(node, topic_name: str, qos_args):
    if (qos_args.qos_reliability is not None or
            qos_args.qos_durability is not None or
            qos_args.qos_depth is not None or
            qos_args.qos_history is not None or
            qos_args.qos_liveliness is not None or
            qos_args.qos_liveliness_lease_duration_seconds is not None):

        return qos_profile_from_short_keys(
            qos_args.qos_profile,
            reliability=qos_args.qos_reliability,
            durability=qos_args.qos_durability,
            depth=qos_args.qos_depth,
            history=qos_args.qos_history,
            liveliness=qos_args.qos_liveliness,
            liveliness_lease_duration_s=qos_args.qos_liveliness_lease_duration_seconds)

    qos_profile = QoSPresetProfiles.get_from_short_key(qos_args.qos_profile)
    reliability_reliable_endpoints_count = 0
    durability_transient_local_endpoints_count = 0

    pubs_info = node.get_publishers_info_by_topic(topic_name)
    publishers_count = len(pubs_info)
    if publishers_count == 0:
        return qos_profile

    for info in pubs_info:
        if (info.qos_profile.reliability == QoSReliabilityPolicy.RELIABLE):
            reliability_reliable_endpoints_count += 1
        if (info.qos_profile.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL):
            durability_transient_local_endpoints_count += 1

    # If all endpoints are reliable, ask for reliable
    if reliability_reliable_endpoints_count == publishers_count:
        qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
    else:
        if reliability_reliable_endpoints_count > 0:
            print(
                'Some, but not all, publishers are offering '
                'QoSReliabilityPolicy.RELIABLE. Falling back to '
                'QoSReliabilityPolicy.BEST_EFFORT as it will connect '
                'to all publishers'
            )
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

    # If all endpoints are transient_local, ask for transient_local
    if durability_transient_local_endpoints_count == publishers_count:
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    else:
        if durability_transient_local_endpoints_count > 0:
            print(
                'Some, but not all, publishers are offering '
                'QoSDurabilityPolicy.TRANSIENT_LOCAL. Falling back to '
                'QoSDurabilityPolicy.VOLATILE as it will connect '
                'to all publishers'
            )
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

    return qos_profile
