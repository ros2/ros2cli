# Copyright 2026 Peng Wang
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


import os
from unittest.mock import patch

import colorama
import pytest

from ros2cli.color import black
from ros2cli.color import blue
from ros2cli.color import bold
from ros2cli.color import ColorState
from ros2cli.color import cyan
from ros2cli.color import dim
from ros2cli.color import green
from ros2cli.color import magenta
from ros2cli.color import red
from ros2cli.color import underline
from ros2cli.color import white
from ros2cli.color import yellow


@pytest.fixture(autouse=True)
def reset_color_state():
    """Reset ColorState after every test."""
    yield
    ColorState.reset()


class TestColorStateDefault:
    """Color is disabled by default (no env var, no --color flag)."""

    def test_disabled_by_default(self):
        with patch.dict(os.environ, {}, clear=True):
            ColorState.reset()
            assert ColorState.is_enabled() is False

    def test_disabled_when_env_var_absent(self):
        env = {k: v for k, v in os.environ.items() if k != 'ROS2CLI_COLOR_OUTPUT'}
        with patch.dict(os.environ, env, clear=True):
            ColorState.reset()
            assert ColorState.is_enabled() is False


class TestRosColorOutput:
    """Test ROS2CLI_COLOR_OUTPUT environment variable (requires TTY)."""

    def test_enabled_when_set_to_1_with_tty(self):
        with patch.dict(os.environ, {'ROS2CLI_COLOR_OUTPUT': '1'}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': True}):
                ColorState.reset()
                assert ColorState.is_enabled() is True

    def test_disabled_when_set_to_1_without_tty(self):
        with patch.dict(os.environ, {'ROS2CLI_COLOR_OUTPUT': '1'}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': False}):
                ColorState.reset()
                assert ColorState.is_enabled() is False

    def test_enabled_for_truthy_values_with_tty(self):
        for val in ('1', '2', 'true', 'yes'):
            with patch.dict(os.environ, {'ROS2CLI_COLOR_OUTPUT': val}, clear=True):
                with patch('sys.stdout', **{'isatty.return_value': True}):
                    ColorState.reset()
                    assert ColorState.is_enabled() is True, \
                        f'ROS2CLI_COLOR_OUTPUT={val!r} with TTY should enable color'  # noqa: E501

    def test_disabled_for_zero(self):
        with patch.dict(os.environ, {'ROS2CLI_COLOR_OUTPUT': '0'}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': True}):
                ColorState.reset()
                assert ColorState.is_enabled() is False

    def test_disabled_for_empty_string(self):
        with patch.dict(os.environ, {'ROS2CLI_COLOR_OUTPUT': ''}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': True}):
                ColorState.reset()
                assert ColorState.is_enabled() is False


class TestSetFromArgs:
    """Test ColorState.set_from_args() — the --color flag."""

    def test_flag_true_enables_color_with_tty(self):
        with patch.dict(os.environ, {}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': True}):
                ColorState.reset()
                ColorState.set_from_args(True)
                assert ColorState.is_enabled() is True

    def test_flag_true_disabled_without_tty(self):
        with patch.dict(os.environ, {}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': False}):
                ColorState.reset()
                ColorState.set_from_args(True)
                assert ColorState.is_enabled() is False

    def test_flag_false_falls_back_to_env_enabled_with_tty(self):
        with patch.dict(os.environ, {'ROS2CLI_COLOR_OUTPUT': '1'}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': True}):
                ColorState.reset()
                ColorState.set_from_args(False)
                assert ColorState.is_enabled() is True

    def test_flag_false_falls_back_to_env_disabled(self):
        with patch.dict(os.environ, {}, clear=True):
            ColorState.reset()
            ColorState.set_from_args(False)
            assert ColorState.is_enabled() is False


class TestAtomicEnabled:
    """Test each atomic color/style function when color is enabled."""

    def setup_method(self):
        ColorState._enabled = True

    def _check(self, func, *expected_codes, text='hello'):
        result = func(text)
        for code in expected_codes:
            assert code in result
        assert text in result
        assert result.endswith(colorama.Style.RESET_ALL)

    def test_bold(self):     self._check(bold,    colorama.Style.BRIGHT)
    def test_dim(self):      self._check(dim,     colorama.Style.DIM)
    def test_black(self):    self._check(black,   colorama.Fore.BLACK)
    def test_red(self):      self._check(red,     colorama.Fore.RED)
    def test_green(self):    self._check(green,   colorama.Fore.GREEN)
    def test_yellow(self):   self._check(yellow,  colorama.Fore.YELLOW)
    def test_blue(self):     self._check(blue,    colorama.Fore.BLUE)
    def test_magenta(self):  self._check(magenta, colorama.Fore.MAGENTA)
    def test_cyan(self):     self._check(cyan,    colorama.Fore.CYAN)
    def test_white(self):    self._check(white,   colorama.Fore.WHITE)

    def test_underline(self):
        result = underline('x')
        assert 'x' in result and result.endswith(colorama.Style.RESET_ALL)

    def test_reset_appended_once(self):
        result = bold('x')
        assert result.endswith(colorama.Style.RESET_ALL)
        assert result.count(colorama.Style.RESET_ALL) == 1


class TestAtomicDisabled:
    """Test each atomic color/style function is transparent when color is disabled."""

    def setup_method(self):
        ColorState._enabled = False

    def test_all_transparent(self):
        text = 'test'
        for func in (bold, dim, underline, black, red, green, yellow, blue,
                     magenta, cyan, white):
            result = func(text)
            assert result == text, f'{func.__name__}() should be transparent when disabled'
            assert '\x1b' not in result


class TestNesting:
    """Test composition of color functions (nesting)."""

    def setup_method(self):
        ColorState._enabled = True

    def test_bold_cyan(self):
        result = bold(cyan('/my_node'))
        assert colorama.Style.BRIGHT in result
        assert colorama.Fore.CYAN in result
        assert '/my_node' in result

    def test_nesting_disabled_is_transparent(self):
        ColorState._enabled = False
        assert bold(cyan('x')) == 'x'

    def test_bold_yellow(self):
        result = bold(yellow('WARNING'))
        assert colorama.Style.BRIGHT in result
        assert colorama.Fore.YELLOW in result


class TestEnvVarIntegration:
    """Integration tests for env var and --color flag end-to-end."""

    def test_default_no_color(self):
        with patch.dict(os.environ, {}, clear=True):
            ColorState.reset()
            result = bold(cyan('/my_node'))
        assert result == '/my_node'

    def test_ros_color_output_colorizes_with_tty(self):
        with patch.dict(os.environ, {'ROS2CLI_COLOR_OUTPUT': '1'}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': True}):
                ColorState.reset()
                result = bold(cyan('/my_node'))
        assert '\x1b' in result
        assert '/my_node' in result

    def test_ros_color_output_no_color_without_tty(self):
        with patch.dict(os.environ, {'ROS2CLI_COLOR_OUTPUT': '1'}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': False}):
                ColorState.reset()
                result = bold(cyan('/my_node'))
        assert result == '/my_node'

    def test_color_flag_colorizes_with_tty(self):
        with patch.dict(os.environ, {}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': True}):
                ColorState.reset()
                ColorState.set_from_args(True)
                result = bold(cyan('/my_node'))
        assert '\x1b' in result
        assert '/my_node' in result

    def test_color_flag_no_color_without_tty(self):
        with patch.dict(os.environ, {}, clear=True):
            with patch('sys.stdout', **{'isatty.return_value': False}):
                ColorState.reset()
                ColorState.set_from_args(True)
                result = bold(cyan('/my_node'))
        assert result == '/my_node'
