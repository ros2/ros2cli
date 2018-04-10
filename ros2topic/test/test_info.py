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

from argparse import Namespace
from contextlib import contextmanager
from io import StringIO
import sys

from ros2topic.verb.info import InfoVerb

NODE_NAME = 'bar'
TOPIC_NAME = '/foo'


@contextmanager
def string_stdout() -> StringIO:
    real_stdout = sys.stdout
    string_io_stdout = StringIO()
    sys.stdout = string_io_stdout
    yield string_io_stdout
    sys.stdout = real_stdout


def _generate_expected_output(topic_name, count_publishers, count_subscribers):
    return [
        'Topic: %s' % topic_name,
        'Publishers count: %d' % count_publishers,
        'Subscribers count: %d' % count_subscribers,
    ]


def test_info_zero_publishers_subscribers():
    args = Namespace()
    args.topic_name = TOPIC_NAME
    with string_stdout() as s:
        info_verb = InfoVerb()
        info_verb.main(args=args)
        expected_output = _generate_expected_output(args.topic_name, 0, 0)
        assert expected_output == s.getvalue().splitlines()
