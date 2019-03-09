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

from ros2cli.node.direct import DirectNode
from ros2topic.api import TopicNameCompleter
from ros2topic.api import get_topic_names_and_types
from ros2topic.verb import VerbExtension


class InfoVerb(VerbExtension):
    """Print information about a topic."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to get info (e.g. '/chatter')")
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')

    def main(self, *, args):
        with DirectNode(args) as node:
            topic_name = args.topic_name
            type_name = "NOT FOUND!"

            all_topic_names_and_types = get_topic_names_and_types(node=node, include_hidden_topics=True)
            for topic, type in all_topic_names_and_types:
                if topic == topic_name:
                    if len(type) > 1:
                        raise RuntimeError(
                            "Topic '%s', contains more than one type: [%s]" %
                            (topic_name, ', '.join(type))
                        )
                    else:
                        type_name = type[0]
                    break

            print('Topic: %s' % topic_name)
            print('Type: %s' % type_name)
            print('Publisher count: %d' % node.count_publishers(topic_name))
            print('Subscriber count: %d' % node.count_subscribers(topic_name))
