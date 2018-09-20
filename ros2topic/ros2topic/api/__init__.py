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

from rclpy.topic_or_service_is_hidden import topic_or_service_is_hidden
from ros2cli.node.strategy import NodeStrategy
from ros2msg.api import message_type_completer


def get_topic_names_and_types(*, node, include_hidden_topics=False):
    topic_names_and_types = node.get_topic_names_and_types()
    if not include_hidden_topics:
        topic_names_and_types = [
            (n, t) for (n, t) in topic_names_and_types
            if not topic_or_service_is_hidden(n)]
    return topic_names_and_types


def get_topic_names(*, node, include_hidden_topics=False):
    topic_names_and_types = get_topic_names_and_types(
        node=node, include_hidden_topics=include_hidden_topics)
    return [n for (n, t) in topic_names_and_types]


class TopicNameCompleter:
    """Callable returning a list of topic names."""

    def __init__(self, *, include_hidden_topics_key=None):
        self.include_hidden_topics_key = include_hidden_topics_key

    def __call__(self, prefix, parsed_args, **kwargs):
        with NodeStrategy(parsed_args) as node:
            return get_topic_names(
                node=node,
                include_hidden_topics=getattr(
                    parsed_args, self.include_hidden_topics_key))


class TopicTypeCompleter:
    """Callable returning an existing topic type or all message types."""

    def __init__(self, *, topic_name_key=None):
        self.topic_name_key = topic_name_key

    def __call__(self, prefix, parsed_args, **kwargs):
        if self.topic_name_key is not None:
            with NodeStrategy(parsed_args) as node:
                topic_name = getattr(parsed_args, self.topic_name_key)
                names_and_types = get_topic_names_and_types(
                    node=node, include_hidden_topics=True)
                for n, t in names_and_types:
                    if n == topic_name:
                        return t
        return message_type_completer()


class SetFieldError(Exception):

    def __init__(self, field_name, exception):
        super(SetFieldError, self).__init__()
        self.field_name = field_name
        self.exception = exception


def set_msg_fields(msg, values, keys={}):
    for field_name, field_value in values.items():
        # check for substitution key, e.g. 'now'
        if type(field_value) == str:
            if field_value in keys:
                try:
                    setattr(msg, field_name, keys[field_value])
                except Exception as e:
                    raise SetFieldError(field_name, e)
                continue

        field_type = type(getattr(msg, field_name))
        try:
            value = field_type(field_value)
        except TypeError:
            value = field_type()
            try:
                set_msg_fields(value, field_value, keys=keys)
            except SetFieldError as e:
                raise SetFieldError(
                    '{field_name}.{e.field_name}'.format_map(locals()),
                    e.exception)
        except ValueError as e:
            raise SetFieldError(field_name, e)
        try:
            setattr(msg, field_name, value)
        except Exception as e:
            raise SetFieldError(field_name, e)
