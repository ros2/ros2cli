# Copyright 2026 Sylvester Kaczmarek
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

from types import SimpleNamespace

import pytest

from ros2topic.verb.delay import _get_message_field


def test_get_message_field_returns_root_message_without_field():
    msg = SimpleNamespace(header=object())

    assert _get_message_field(msg, None) is msg


def test_get_message_field_supports_nested_array_field():
    transform = SimpleNamespace(header=object())
    msg = SimpleNamespace(transforms=[transform])

    assert _get_message_field(msg, 'transforms.[0]') is transform


def test_get_message_field_rejects_invalid_field():
    msg = SimpleNamespace(transforms=[])

    with pytest.raises(RuntimeError, match="Invalid field 'transforms.\\[0\\]'"):
        _get_message_field(msg, 'transforms.[0]')
