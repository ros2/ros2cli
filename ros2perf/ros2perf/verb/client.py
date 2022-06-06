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

from ros2perf.api import add_qos_arguments_to_parser
from ros2perf.api import get_qos_profile_from_args
from ros2perf.api import nonnegative_float
from ros2perf.api import positive_float
from ros2perf.api import positive_int
from ros2perf.api import print_results
from ros2perf.api import print_stats_header
from ros2perf.verb import VerbExtension


class ClientVerb(VerbExtension):
    """Run client side of ros2 performace tester."""

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
        parser.add_argument(
            '-d',
            '--duration',
            help='Duration of the experiment to be run in seconds.',
            type=positive_float
        )
        add_qos_arguments_to_parser(parser)


    def main(self, *, args):
        qos_profile = get_qos_profile_from_args(args)
        # pub_period[sec] = 8[b/Byte] * message_size[Byte] * 1[M]/10^6 / target_bandwidth[Mb/s]
        pub_period = 8 * args.message_size / 1e6 / args.target_bw

        runner = perf_tool.ClientRunner(
            experiment_duration_s=args.duration,
            qos=qos_profile,
            message_size_bytes=args.message_size,
            target_pub_period_s=pub_period)
        try:
            with runner as node:
                print('---------------------------------------------------------')
                print('Client running')
                print(f'\ttopic: {node.get_topic_name()}')
                print(f'\tpublisher gid: {node.get_stringified_pub_gid()}')
                print(f'\tqos: {qos_profile}')
                print('---------------------------------------------------------')
                runner.wait_for_experiment_to_complete()
        except KeyboardInterrupt:
            pass
        except:
            raise
        try:
            results = node.extract_results()
        except:
            raise

        print_stats_header()
        print_results(results.statistics, id=0)
