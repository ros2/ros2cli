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

from ros2topic.verb.delay import ROSTopicDelay


class _Node:

    def get_clock(self):
        return object()


def _delay_output(capsys, precision=3):
    delay = ROSTopicDelay(_Node(), window_size=10, precision=precision)
    delay.delays = [1234567, 2345678]
    delay.msg_tn = 1

    delay.print_delay()

    return capsys.readouterr().out


def test_default_delay_precision(capsys):
    output = _delay_output(capsys)

    assert 'average delay: 0.002\n' in output
    assert '\tmin: 0.001s max: 0.002s' in output
    assert 'std dev: 0.00056s window: 2' in output


def test_custom_delay_precision(capsys):
    output = _delay_output(capsys, precision=6)

    assert 'average delay: 0.001790\n' in output
    assert '\tmin: 0.001235s max: 0.002346s' in output
    assert 'std dev: 0.00056s window: 2' in output
