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

import netperf_tool

from ros2netperf.api import add_qos_arguments_to_parser
from ros2netperf.api import get_qos_profile_from_args
from ros2netperf.api import print_results
from ros2netperf.api import print_stats_header
from ros2netperf.verb import VerbExtension


class ServerVerb(VerbExtension):
    """Run server side of ros2 performance tester."""

    def __init__(self):
        self.next_client_id_ = 0
        self.gid_id_map_ = {}

    def add_arguments(self, parser, cli_name):
        add_qos_arguments_to_parser(parser)


    def main(self, *, args):
        qos_profile = get_qos_profile_from_args(args)

        runner = netperf_tool.ServerRunner(qos=qos_profile)
        try:
            with runner as node:
                print('---------------------------------------------------------')
                print('Server running')
                print(f'\ttopic: {node.get_topic_name()}')
                print(f'\tqos: {qos_profile}')
                print('---------------------------------------------------------')
                while True:
                    # we wait for 1 second, so we can check for signals in the middle
                    # as the blocking method used by wait_for_results_available()
                    # isn't awaken by signals.
                    self.print_events_from_node(node)
        except KeyboardInterrupt:
            pass
        # if there were more results available, print them
        self.print_events_from_node(node)


    def print_events_from_node(self, node):
        node.wait_for_results_available(1.)
        clients_gids = node.extract_new_clients()
        for client_gid in clients_gids:
            self.gid_id_map_[client_gid] = self.next_client_id_
            print(f'[ {self.next_client_id_}] Publisher with gid [{client_gid}] connected')
            self.next_client_id_ = self.next_client_id_ + 1
        results_map = node.extract_results()
        self.print_results(results_map)


    def print_results(self, results_map):
        # TODO(ivanpauno): Add some processing to be able to show better statistics
        if 0 == len(results_map):
            return
        print_stats_header()
        for gid, results in results_map.items():
            id = self.gid_id_map_[gid]
            print_results(results.statistics, id=id)
