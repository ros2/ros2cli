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

import argparse

from geometry_msgs.msg import Twist
import pytest
from ros2topic.filter_expression import compile_filter, FilterExpressionError
from ros2topic.verb.hz import main as hz_main
from std_msgs.msg import String
from test_msgs.msg import Arrays


def evaluate(expression, message):
    return compile_filter(expression)(message)


@pytest.mark.parametrize('expression,expected', [
    ('m.data == "chatter: 4"', True),
    ('int(m.data.rpartition(":")[-1]) % 2 == 0', True),
    ('m.data.startswith("chat") and not m.data.endswith("x")', True),
    ('len(m.data) > 3', True),
    ('m.data[0] == "c"', True),
    ('m.data[:4] == "chat"', True),
    ('1 < len(m.data) < 5', False),
    ('"chat" in m.data', True),
    ('m.data in ("a", "b")', False),
    ('"yes" if m.data else "no"', 'yes'),
    ('2 ** 3 + 1 * 2 - 4 // 3', 9),
    ('abs(-3) + max([], default=5)', 8),
])
def test_string_message(expression, expected):
    assert evaluate(expression, String(data='chatter: 4')) == expected


def test_nested_fields():
    msg = Twist()
    msg.linear.x = 2.0
    assert evaluate('m.linear.x > 1.5 and m.angular.z == 0.0', msg)
    assert not evaluate('m.linear.x < 1.5', msg)


def test_sequence_fields_and_comprehensions():
    msg = Arrays()
    msg.string_values = ['a', 'bb', 'ccc']
    msg.float32_values = [1.0, 2.0, 3.0]
    assert evaluate('any(x > 2 for x in m.float32_values)', msg)
    assert not evaluate('all(x > 2 for x in m.float32_values)', msg)
    assert evaluate('len([s for s in m.string_values if len(s) > 1]) == 2', msg)
    assert evaluate('sum(len(s) for s in m.string_values) == 6', msg)
    assert evaluate('{s: len(s) for s in m.string_values}["bb"] == 2', msg)
    assert evaluate('[a + b for a in "xy" for b in "12" if b != "2"] == ["x1", "y1"]', msg)
    assert evaluate('[a for a, b in [(1, 2), (3, 4)] if b > 2] == [3]', msg)
    assert evaluate('m.string_values.count("a") == 1', msg)


def test_comprehension_variables_do_not_leak():
    with pytest.raises(FilterExpressionError, match="Name 'x' is not defined"):
        evaluate('any([x for x in [1]]) and x', String())


@pytest.mark.parametrize('expression', [
    '__import__("os").system("true")',
    'open("/etc/passwd")',
    'eval("1")',
    'getattr(m, "data")',
    'type(m)',
    'isinstance(m, str)',
    'print(1)',
    '__builtins__["open"]("/etc/passwd")',
    'm.__class__',
    'm.__class__.__mro__',
    'm._data',
    'm.data.__class__',
    '(lambda: 1)()',
    'f"{m.data}"',
    '(x := 1)',
    '[*m.data]',
    'max(*[1, 2])',
    'dict(**{"a": 1})',
    '{**{"a": 1}}',
    'm.data.upper().__class__',
    '[x for x in m.data].__class__',
    '().__class__',
    'b"abc"',
    '...',
    'm.data if (yield) else 0',
    '1; 2',
    'import os',
    '1 +',
    '1\x00',
    '"\ud800"',
    '-' * 5000 + '1',
    '+'.join(['1'] * 5000),
    'not ' * 5000 + '1',
    'm.data' + '[0]' * 3000,
    '(' * 500 + '1' + ')' * 500,
])
def test_rejected_at_compile_time(expression):
    with pytest.raises(FilterExpressionError):
        compile_filter(expression)


@pytest.mark.parametrize('expression', [
    'm.nonexistent',
    'os',
    '__builtins__',
    'm.get_fields_and_field_types',
    'm.data.format',
    '"{0.__class__}".format(m)',
    '"{}".format_map({})',
    '"".join(["a"])',
    'm.data.encode()',
    'str.upper("a")',
    '1 .real',
    'm.data.nonexistent',
    'm.data.upper().format',
    'm.data.bit_length()',
    'm.data[0].bit_length()',
    'm.data()',
    'undefined_name',
    '1 / 0',
    'm.data + 1',
    'm["data"]',
    '2 ** 100000',
    '"a" * 100000000',
    '[0] * 100000000',
    '"a" * 1000 * 1000',
    '[0] * 1000 * 1000',
    '(2 ** 1000) ** 1000',
    'int("9" * 4000) ** 1000',
    '"%s" % m.data',
    '"%1000000000s" % 1',
    '"a".replace("a", "b")',
    'sum(1 for a in "x" * 1000 for b in "x" * 1000)',
    'sum(1 for a in "x" * 1000 for b in "x" * 1000 for c in "x" * 1000)',
    '[1 for a in "x" * 500 if [1 for b in "x" * 500]]',
])
def test_rejected_at_runtime(expression):
    with pytest.raises(FilterExpressionError):
        evaluate(expression, String(data='chatter: 4'))


def test_hz_reports_invalid_filter_without_running():
    args = argparse.Namespace(filter_expr='__import__("os").system("true")')
    result = hz_main(args)
    assert isinstance(result, str)
    assert 'not allowed' in result
