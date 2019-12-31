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

from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSLivelinessPolicy
from rclpy.qos import QoSReliabilityPolicy

from ros2cli.node.direct import DirectNode
from ros2topic.api import get_topic_names_and_types
from ros2topic.api import TopicNameCompleter
from ros2topic.verb import VerbExtension


def print_topic_info(topic_name, get_topic_info_func):
    for info in get_topic_info_func(topic_name):
        print('Node name: %s' % info['node_name'])
        print('Node namespace: %s' % info['node_namespace'])
        print('Topic type: %s' % info['topic_type'])
        print('GID: %s' % '.'.join(format(x, '02x') for x in info['gid']))
        print('QoS profile:')
        qos_profile = info['qos_profile']
        print('  Reliability: %s' % QoSReliabilityPolicy(qos_profile['reliability']).name)
        print('  Durability: %s' % QoSDurabilityPolicy(qos_profile['durability']).name)
        print('  Lifespan: %d nanoseconds' % qos_profile['lifespan'].nanoseconds)
        print('  Deadline: %d nanoseconds' % qos_profile['deadline'].nanoseconds)
        print('  Liveliness: %s' % QoSLivelinessPolicy(qos_profile['liveliness']).name)
        print('  Liveliness lease duration: %d nanoseconds\n' %
              qos_profile['liveliness_lease_duration'].nanoseconds)

class InfoVerb(VerbExtension):
    """Print information about a topic."""

    def add_arguments(self, parser, cli_name):
        arg = parser.add_argument(
            'topic_name',
            help="Name of the ROS topic to get info (e.g. '/chatter')")
        parser.add_argument(
            '--verbose', '-v', action='store_true',
            help='Prints detailed information like the node name, node namespace, topic type, '
                 'GUID and QoS Profile of the publishers and subscribers to this topic')
        arg.completer = TopicNameCompleter(
            include_hidden_topics_key='include_hidden_topics')

    def main(self, *, args):
        with DirectNode(args) as node:
            topic_names_and_types = get_topic_names_and_types(
                node=node, include_hidden_topics=True)
            topic_name = args.topic_name
            for (t_name, t_types) in topic_names_and_types:
                if t_name == topic_name:
                    topic_types = t_types
                    break
            else:
                return "Unknown topic '%s'" % topic_name

            lineend = '\n'
            if args.verbose:
                lineend = '\n\n'

            type_str = topic_types[0] if len(topic_types) == 1 else topic_types
            print('Type: %s' % type_str, end=lineend)

            print('Publisher count: %d' % node.count_publishers(topic_name), end=lineend)
            if args.verbose:
                try:
                    print_topic_info(topic_name, node.get_publishers_info_by_topic)
                except NotImplementedError as e:
                    return str(e)

            print('Subscription count: %d' % node.count_subscribers(topic_name))
            if args.verbose:
                try:
                    print_topic_info(topic_name, node.get_subscriptions_info_by_topic)
                except NotImplementedError as e:
                    return str(e)
