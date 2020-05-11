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

from ros2cli.node.strategy import NodeStrategy

from ros2topic.api import get_topic_names_and_types
from ros2topic.verb import VerbExtension


class ListVerb(VerbExtension):
    """Output a list of available topics."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '-t', '--show-types', action='store_true',
            help='Additionally show the topic type')
        parser.add_argument(
            '-c', '--count-topics', action='store_true',
            help='Only display the number of topics discovered')
        # duplicate the following argument from the command for visibility
        parser.add_argument(
            '--include-hidden-topics', action='store_true',
            help='Consider hidden topics as well')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            topic_names_and_types = get_topic_names_and_types(
                node=node,
                include_hidden_topics=args.include_hidden_topics)

        if args.count_topics:
            print(len(topic_names_and_types))
        elif topic_names_and_types:
            for (topic_name, topic_types) in topic_names_and_types:
                msg = f'{topic_name}'
                topic_types_formatted = ', '.join(topic_types)
                if args.show_types:
                    msg += f' [{topic_types_formatted}]'
                print(msg)
