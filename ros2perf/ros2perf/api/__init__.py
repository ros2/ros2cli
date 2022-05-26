# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSPresetProfiles
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from ros2topic.api import profile_configure_short_keys


def positive_int(inval):
    # Convert first to float, to support scientific notation, like 10e3
    ret = float(inval)
    if not ret.is_integer():
        raise ValueError('Value must be integer')
    ret = int(ret)
    if ret <= 0:
        raise ValueError('Value must be positive')
    return ret


def positive_float(inval):
    ret = float(inval)
    if ret <= 0.0:
        raise ValueError('Value must be positive')
    return ret


def nonnegative_float(inval):
    ret = float(inval)
    if ret < 0.0:
        raise ValueError('Value must be positive or zero')
    return ret


def get_default_qos_profile():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=1)


def add_qos_arguments_to_parser(parser):
    parser.add_argument(
        '--qos-profile',
        choices=QoSPresetProfiles.short_keys(),
        help='Quality of service preset profile to publish')
    default_profile = get_default_qos_profile()
    parser.add_argument(
        '--qos-depth', metavar='N', type=int, default=-1,
        help='Queue size setting to publish with '
                '(overrides depth value of --qos-profile option)')
    parser.add_argument(
        '--qos-history',
        choices=QoSHistoryPolicy.short_keys(),
        help='History of samples setting to publish with '
                '(overrides history value of --qos-profile option, default: {})'
                .format(default_profile.history.short_key))
    parser.add_argument(
        '--qos-reliability',
        choices=QoSReliabilityPolicy.short_keys(),
        help='Quality of service reliability setting to publish with '
                '(overrides reliability value of --qos-profile option, default: {})'
                .format(default_profile.reliability.short_key))
    parser.add_argument(
        '--qos-durability',
        choices=QoSDurabilityPolicy.short_keys(),
        help='Quality of service durability setting to publish with '
                '(overrides durability value of --qos-profile option, default: {})'
                .format(default_profile.durability.short_key))


def get_qos_profile_from_args(args):
    qos_profile = get_default_qos_profile()

    qos_profile_name = args.qos_profile
    if qos_profile_name:
        qos_profile = QoSPresetProfiles.get_from_short_key(qos_profile_name)
    profile_configure_short_keys(
        qos_profile, args.qos_reliability, args.qos_durability,
        args.qos_depth, args.qos_history)
    return qos_profile
