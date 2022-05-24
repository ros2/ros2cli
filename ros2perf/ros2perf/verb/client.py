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

import perf_tool

import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from ros2topic.api import profile_configure_short_keys

from ros2perf.verb import VerbExtension


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


def get_pub_qos_profile():
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=1)

class ClientVerb(VerbExtension):
    """Output information about a node."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-s',
            '--message-size',
            help='Message size to be used in bytes',
            type=positive_int
        )
        parser.add_argument(
            '-b',
            '--target-bw',
            help='Target bandwidth in Mbps. The client will use a publishing rate '
            'to achieve the desired bandwidth utilization. '
            'If zero (0), the client will publish as fast as possible.',
            type=nonnegative_float
        )
        # TODO(ivanpauno): Add qos arguments!
        parser.add_argument(
            '-d',
            '--duration',
            help='Duration of the experiment to be run in seconds.',
            type=positive_float
        )
        parser.add_argument(
            '--qos-profile',
            choices=rclpy.qos.QoSPresetProfiles.short_keys(),
            help='Quality of service preset profile to publish')
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


    def main(self, *, args):
        qos_profile = get_pub_qos_profile()

        qos_profile_name = args.qos_profile
        if qos_profile_name:
            qos_profile = rclpy.qos.QoSPresetProfiles.get_from_short_key(qos_profile_name)
        profile_configure_short_keys(
            qos_profile, args.qos_reliability, args.qos_durability,
            args.qos_depth, args.qos_history)
        # pub_period[sec] = 8[b/Byte] * message_size[Byte] * 1M / target_bandwidth[Mb/s]
        pub_period = 8 * args.message_size * 10e6 / args.target_bw
        results = perf_tool.run_client(
            args.message_size, pub_period, qos_profile.get_c_qos_profile(), args.duration)
        
