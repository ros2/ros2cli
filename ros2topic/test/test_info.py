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

import sys
from io import StringIO
from argparse import Namespace
from contextlib import contextmanager

from ros2topic.verb.info import InfoVerb


@contextmanager
def string_stdout() -> StringIO:
    real_stdout = sys.stdout
    stringio_stdout = StringIO()
    sys.stdout = stringio_stdout
    yield stringio_stdout
    sys.stdout = real_stdout


def test_info():
    args = Namespace()
    args.topic_name = 'foo'
    with string_stdout() as s:
        info_verb = InfoVerb()
        info_verb.main(args=args)
        assert args.topic_name == s.getvalue().splitlines()[0]
