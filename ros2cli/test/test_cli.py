# Copyright 2026 Open Source Robotics Foundation, Inc.
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

import io
from unittest.mock import patch

from ros2cli import cli


class _BrokenPipeExtension:

    def add_arguments(self, parser, script_name):
        pass

    def main(self, *, parser, args):
        raise BrokenPipeError(32, 'Broken pipe')


class _OkExtension:

    def add_arguments(self, parser, script_name):
        pass

    def main(self, *, parser, args):
        return 0


def test_broken_pipe_error_exits_quietly():
    # sys.stdout without a file descriptor: the /dev/null redirect is skipped.
    stdout = io.TextIOWrapper(io.BytesIO())
    with patch('sys.stdout', stdout):
        rc = cli.main(argv=[], extension=_BrokenPipeExtension())
    assert rc == 1


def test_normal_extension_return_code_unchanged():
    stdout = io.TextIOWrapper(io.BytesIO())
    with patch('sys.stdout', stdout):
        rc = cli.main(argv=[], extension=_OkExtension())
    assert rc == 0
