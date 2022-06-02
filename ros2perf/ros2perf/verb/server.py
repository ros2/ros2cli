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
from ros2perf.verb import VerbExtension


class ServerVerb(VerbExtension):
    """Run server side of ros2 performance tester."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-d',
            '--duration',
            help='Duration of the experiment to be run in seconds.',
            type=positive_float
        )
        add_qos_arguments_to_parser(parser)


    def main(self, *, args):
        qos_profile = get_qos_profile_from_args(args)

        runner = perf_tool.ServerRunner(
            qos_profile.get_c_qos_profile(), args.duration)
        try:
            with runner as node:
                print('---------------------------------------------------------')
                print('Server running')
                print(f'\ttopic: {node.get_topic_name()}')
                print(f'\tqos: {qos_profile}')
                print('---------------------------------------------------------')
                runner.wait_for_experiment_to_complete()
        except KeyboardInterrupt:
            pass
        results_map = node.extract_results()

        # TODO(ivanpauno): Add some processing to be able to show better statistics
        for gid, results in results_map.items():
            print(gid)
            print(results.statistics.latency_avg_ms)
            print(results.statistics.latency_stdev_ms)
            print(results.statistics.latency_min_ms)
            print(results.statistics.latency_max_ms)
            print(results.statistics.total_bytes)
            print(results.statistics.experiment_duration_ns)
            print(results.statistics.messages_lost)
            print(results.statistics.messages_total)

            print(results.collected_info.message_ids)
            print(results.collected_info.message_latencies)
            print(results.collected_info.message_sizes)
