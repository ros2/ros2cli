# Copyright 2019 Canonical Ltd.
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
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension


class TypeVerb(VerbExtension):
    """Print a topic's type."""

    def add_arguments(self, parser, cli_name):
        add_strategy_node_arguments(parser)

        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to get type (e.g. '/chatter')")
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')

    def main(self, *, args):
        with NodeStrategy(args) as node:
            topic_names_and_types = get_topic_names_and_types(
                node=node,
                include_hidden_topics=args.include_hidden_topics)

        for (topic_name, topic_types) in topic_names_and_types:
            if args.topic_name == topic_name:
                for topic_type in topic_types:
                    print(topic_type)
                return 0

        return 1
