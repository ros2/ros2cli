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

from ros2cli.node.strategy import add_arguments as add_strategy_node_arguments
from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import get_topic_names_and_types
from ros2topic.verb import VerbExtension


class ListVerb(VerbExtension):
    """Output a list of available topics."""

    def add_arguments(self, parser, cli_name):
        add_strategy_node_arguments(parser)

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
        parser.add_argument(
            '-v', '--verbose', action='store_true',
            help='List full details about each topic')

    @staticmethod
    def show_topic_info(topic_info, is_publisher):
        message = ('Published' if is_publisher else 'Subscribed') + ' topics:\n'
        for (topic_name, topic_types, pub_count, sub_count) in topic_info:
            count = pub_count if is_publisher else sub_count
            if count:
                topic_types_formatted = ', '.join(topic_types)
                count_str = str(count) + ' ' + ('publisher' if is_publisher else 'subscriber') \
                    + ('s' if count > 1 else '')
                message += ' * %s [%s] %s\n' % (topic_name, topic_types_formatted, count_str)
        return message

    def main(self, *, args):
        topic_info = []
        with NodeStrategy(args) as node:
            topic_names_and_types = get_topic_names_and_types(
                node=node,
                include_hidden_topics=args.include_hidden_topics)
            for (topic_name, topic_types) in topic_names_and_types:
                pub_count = node.count_publishers(topic_name)
                sub_count = node.count_subscribers(topic_name)
                topic_info.append((topic_name, topic_types, pub_count, sub_count))

        if args.count_topics:
            print(len(topic_names_and_types))
        elif topic_names_and_types:
            if args.verbose:
                print(self.show_topic_info(topic_info, is_publisher=True))
                print(self.show_topic_info(topic_info, is_publisher=False))
            else:
                for (topic_name, topic_types, _, _) in topic_info:
                    msg = '{topic_name}'
                    topic_types_formatted = ', '.join(topic_types)
                    if args.show_types:
                        msg += ' [{topic_types_formatted}]'
                    print(msg.format_map(locals()))
