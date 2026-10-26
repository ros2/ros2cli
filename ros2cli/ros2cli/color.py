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

"""
Color output support for ros2cli.

Built on `colorama <https://pypi.org/project/colorama/>`_ for cross-platform
ANSI support.  Color output is **disabled by default**.

Two ways to enable color:

* ``--color`` command-line flag — enable color for a single invocation.
* ``ROS2CLI_COLOR_OUTPUT`` environment variable set to a non-empty, non-``0``
  value (e.g. ``ROS2CLI_COLOR_OUTPUT=1``) — enable color for all ros2cli
  commands in the current shell session.

In both cases color is only active when stdout is an interactive terminal
(TTY).  Output piped to another command or redirected to a file will never
contain ANSI escape codes.

The ``--color`` flag takes precedence over ``ROS2CLI_COLOR_OUTPUT``.

Usage::

    from ros2cli.color import bold, cyan, yellow, green, red

    print(cyan('/my_node'))
    print(green('/chatter') + ': ' + yellow('std_msgs/msg/String'))
    print(bold(cyan(args.node_name)))

All functions are transparent when color is disabled — they return the
input string unchanged, so call-sites never need to branch on color state.
"""

import os
import sys

import colorama

# Enable ANSI escape processing on Windows (no-op on Linux/macOS).
colorama.just_fix_windows_console()

# colorama does not expose an underline constant; isolate the raw escape here.
_ANSI_UNDERLINE = '\x1b[4m'


class ColorState:
    """
    Manage the global color-enabled flag for ros2cli.

    Color is **disabled by default**.  It can be enabled via the ``--color``
    command-line flag or the ``ROS2CLI_COLOR_OUTPUT`` environment variable.
    """

    _enabled: 'bool | None' = None

    @classmethod
    def is_enabled(cls) -> bool:
        """Return whether color output is currently enabled."""
        if cls._enabled is None:
            cls._enabled = cls._resolve()
        return cls._enabled

    @classmethod
    def _resolve(cls) -> bool:
        """
        Compute the effective color state from the environment.

        Color is enabled when ``ROS2CLI_COLOR_OUTPUT`` is set to a non-empty,
        non-``"0"`` value **and** stdout is a TTY.  The TTY check prevents
        ANSI escape codes from leaking into pipes or redirected files.
        """
        val = os.environ.get('ROS2CLI_COLOR_OUTPUT')
        if val in (None, '', '0'):
            return False
        return cls._is_tty()

    @classmethod
    def _is_tty(cls) -> bool:
        """Return True when stdout is an interactive terminal."""
        try:
            return sys.stdout.isatty()
        except AttributeError:
            return False

    @classmethod
    def set_from_args(cls, color: bool) -> None:
        """Apply the value of the ``--color`` command-line flag."""
        if color:
            cls._enabled = cls._is_tty()

    @classmethod
    def reset(cls) -> None:
        """Reset to undetermined state.  For use in tests only."""
        cls._enabled = None


def _c(text: str, *codes: str) -> str:
    """Apply colorama *codes* to *text*, or return *text* unchanged."""
    if not ColorState.is_enabled():
        return text
    return ''.join(codes) + text + colorama.Style.RESET_ALL


def bold(text: str) -> str:
    """Apply bold/bright style."""
    return _c(text, colorama.Style.BRIGHT)


def dim(text: str) -> str:
    """Apply dim/faint style."""
    return _c(text, colorama.Style.DIM)


def underline(text: str) -> str:
    """Apply underline style."""
    return _c(text, _ANSI_UNDERLINE)


def red(text: str) -> str:
    return _c(text, colorama.Fore.RED)


def green(text: str) -> str:
    return _c(text, colorama.Fore.GREEN)


def yellow(text: str) -> str:
    return _c(text, colorama.Fore.YELLOW)


def blue(text: str) -> str:
    return _c(text, colorama.Fore.BLUE)


def magenta(text: str) -> str:
    return _c(text, colorama.Fore.MAGENTA)


def cyan(text: str) -> str:
    return _c(text, colorama.Fore.CYAN)


def white(text: str) -> str:
    return _c(text, colorama.Fore.WHITE)


def black(text: str) -> str:
    return _c(text, colorama.Fore.BLACK)
