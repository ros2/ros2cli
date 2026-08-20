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

from xmlrpc.client import dumps
from xmlrpc.client import loads

import pytest

import ros2cli.xmlrpc  # noqa: F401


@pytest.mark.parametrize('value', [
    -(1 << 63),
    -(1 << 31) - 1,
    1 << 31,
    (1 << 63) - 1,
])
def test_int64_round_trip(value):
    payload = dumps((value,))

    assert '<i8>' in payload
    assert loads(payload)[0] == (value,)


def test_int32_still_uses_standard_int_tag():
    payload = dumps((42,))

    assert '<int>42</int>' in payload
    assert loads(payload)[0] == (42,)


@pytest.mark.parametrize('value', [-(1 << 63) - 1, 1 << 63])
def test_integer_outside_int64_range_is_rejected(value):
    with pytest.raises(OverflowError, match='XML-RPC i8 limits'):
        dumps((value,))
