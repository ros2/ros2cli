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
Colorized output support for ros2cli.

Colorization is disabled by default.  It is enabled when the environment
variable ROS_COLORIZED_OUTPUT=1 is set and stdout is a tty, or when the
--color flag is passed on the command line.

Architecture
------------
cli.py wraps sys.stdout with ColoringStdout before calling extension.main().
ColoringStdout splits each write() call into lines and passes each non-empty
line through a Colorizer.colorize() method before forwarding to the real
stdout.

The colorizer to use is looked up from a registry keyed by (command, verb)
string pairs (e.g. ('topic', 'echo')).  If no entry is found, DefaultColorizer
is used as a fallback and applies generic heuristics (ROS path names, type
strings, YAML keys).

To register a custom colorizer from another package:

    from ros2cli.color import Colorizer, register

    class MyColorizer(Colorizer):
        def colorize(self, line: str) -> str:
            ...

    register('mycommand', 'myverb', MyColorizer())
"""

import os
import re
import sys

RESET = '\033[0m'
BOLD = '\033[1m'
DIM = '\033[2m'
ITALIC = '\033[3m'
UNDERLINE = '\033[4m'
REVERSE = '\033[7m'

BLACK = '\033[90m'
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
WHITE = '\033[97m'


def is_color_enabled() -> bool:
    """Return True if colorized output should be used."""
    return (
        os.environ.get('ROS_COLORIZED_OUTPUT') == '1'
        and sys.stdout.isatty()
    )


class Colorizer:
    """Base colorizer — passes lines through unchanged."""

    def colorize(self, line: str) -> str:
        return line


class DefaultColorizer(Colorizer):
    """Generic fallback colorizer applied to all unregistered commands."""

    _RE_PATH = re.compile(r'^(/[\w/]+)')
    _RE_TYPE = re.compile(r'([\w]+/(?:msg|srv|action)/[\w]+)')
    _RE_SECTION = re.compile(r'^  \w[\w ]+:$')
    _RE_YAML_KEY = re.compile(r'^(\s*)(\w[\w]*)(:)')

    def colorize(self, line: str) -> str:
        if line == '---':
            return DIM + '---' + RESET

        if self._RE_SECTION.match(line):
            return YELLOW + line + RESET

        # type strings before path to avoid double-coloring
        def _color_type(m):
            return BLUE + m.group(1) + RESET

        colored = self._RE_TYPE.sub(_color_type, line)

        if line.startswith('/'):
            m = self._RE_PATH.match(line)
            if m:
                path = m.group(1)
                rest = self._RE_TYPE.sub(_color_type, line[len(path):])
                return CYAN + path + RESET + rest

        m = self._RE_YAML_KEY.match(line)
        if m:
            indent = m.group(1)
            key = m.group(2)
            colon = m.group(3)
            rest = line[m.end():]
            return indent + CYAN + key + RESET + colon + rest

        return colored


class ColoringStdout:
    """Wraps sys.stdout and colorizes each line before writing."""

    def __init__(self, wrapped, colorizer: Colorizer):
        self._wrapped = wrapped
        self._colorizer = colorizer

    def write(self, text: str) -> int:
        lines = text.split('\n')
        colored = '\n'.join(
            self._colorizer.colorize(line) if line else line for line in lines
        )
        return self._wrapped.write(colored)

    def flush(self):
        self._wrapped.flush()

    def __getattr__(self, name):
        return getattr(self._wrapped, name)


_REGISTRY: dict = {}
_DEFAULT = DefaultColorizer()


def register(command: str, verb: str, colorizer: Colorizer) -> None:
    """Register a colorizer for a specific (command, verb) pair."""
    _REGISTRY[(command, verb)] = colorizer


def get_colorizer(command: str, verb: str) -> Colorizer:
    """Return the registered colorizer, or the default fallback."""
    return _REGISTRY.get((command, verb), _DEFAULT)


class Ros2Colorizer(Colorizer):
    """Colorizer for bare `ros2` command help output."""

    _RE_FLAG = re.compile(r'(--?[\w-]+(?:,\s*--?[\w-]+)*)')

    def colorize(self, line: str) -> str:
        if line.startswith('usage:'):
            line = line.replace('usage:', BOLD + CYAN + 'usage:' + RESET, 1)
            line = line.replace('ros2', BOLD + 'ros2' + RESET, 1)
            line = self._RE_FLAG.sub(BLUE + r'\1' + RESET, line)
            return line

        if '`ros2' in line or 'Call `' in line:
            return DIM + ITALIC + line + RESET

        if line and not line[0].isspace() and line.rstrip().endswith(':'):
            return BOLD + UNDERLINE + YELLOW + line + RESET

        if line.startswith('  -'):
            stripped = line.lstrip()
            m = self._RE_FLAG.match(stripped)
            if m:
                flag_end = line.index(m.group(1)) + len(m.group(1))
                flags = self._RE_FLAG.sub(GREEN + r'\1' + RESET, line[:flag_end])
                rest = line[flag_end:]
                return flags + rest + RESET

        if line.startswith('  ') and not line.startswith('   '):
            parts = line.split(None, 1)
            if parts and parts[0].isalpha():
                raw_rest = line[2 + len(parts[0]):]
                return '  ' + BOLD + CYAN + parts[0] + RESET + WHITE + raw_rest + RESET

        return line


register('', '', Ros2Colorizer())


class TopicEchoColorizer(Colorizer):
    """Colorizer for `ros2 topic echo` — YAML block output."""

    def colorize(self, line: str) -> str:
        if line == '---':
            return DIM + '---' + RESET
        if ':' in line and not line.startswith('-'):
            indent = len(line) - len(line.lstrip())
            key, _, val = line.lstrip().partition(':')
            return ' ' * indent + CYAN + key + RESET + ':' + val
        return line


register('topic', 'echo', TopicEchoColorizer())


class TopicHzColorizer(Colorizer):
    """Colorizer for `ros2 topic hz`."""

    _RE_KV = re.compile(r'(\b(?:min|max|std dev|window)\b)(:\s*)(\S+)')

    def colorize(self, line: str) -> str:
        if line.startswith('average rate:'):
            key, _, val = line.partition(':')
            return BOLD + key + RESET + ':' + CYAN + val + RESET
        if line.startswith('\t'):
            def _color_kv(m):
                return GREEN + m.group(1) + RESET + m.group(2) + CYAN + m.group(3) + RESET
            return self._RE_KV.sub(_color_kv, line)
        return line


register('topic', 'hz', TopicHzColorizer())


class TopicBwColorizer(Colorizer):
    """Colorizer for `ros2 topic bw`."""

    _RE_TITLE = re.compile(r'(\d+.+)(\sfrom\s+\d+\s+messages)')
    _RE_KV = re.compile(r'(\b(?:mean|min|max|window)\b)(:\s*)(\S+)')

    def colorize(self, line: str) -> str:
        if not line.startswith('\t'):
            def _color_title(m):
                return CYAN + m.group(1) + RESET + m.group(2)
            return self._RE_TITLE.sub(_color_title, line)
        if line.startswith('\t'):
            def _color_kv(m):
                return GREEN + m.group(1) + RESET + m.group(2) + CYAN + m.group(3) + RESET
            return self._RE_KV.sub(_color_kv, line)
        return line


register('topic', 'bw', TopicBwColorizer())


class TopicInfoColorizer(Colorizer):
    """Colorizer for `ros2 topic info`."""

    _RE_TYPE = re.compile(r'([\w]+/(?:msg|srv|action)/[\w]+)')

    def colorize(self, line: str) -> str:
        if line.startswith('Type:'):
            key, _, val = line.partition(':')
            def _ct(m): return GREEN + m.group(1) + RESET
            return BOLD + key + RESET + ':' + self._RE_TYPE.sub(_ct, val)
        if line.startswith('Publisher count:') or line.startswith('Subscription count:'):
            key, _, val = line.partition(':')
            return YELLOW + key + RESET + ':' + CYAN + val + RESET
        if line.startswith('  ') and ':' in line:
            indent = len(line) - len(line.lstrip())
            rest = line.lstrip()
            key, _, val = rest.partition(':')
            return ' ' * indent + BLUE + key + RESET + ':' + val
        return line


register('topic', 'info', TopicInfoColorizer())


class ParamGetColorizer(Colorizer):
    """Colorizer for `ros2 param get`."""

    def colorize(self, line: str) -> str:
        if ' value is:' in line or ' value is' in line:
            idx = line.rfind(':')
            if idx != -1:
                label = line[:idx]
                val = line[idx + 1:]
                return YELLOW + label + RESET + ':' + CYAN + val + RESET
        if line.startswith('/') and line.rstrip().endswith(':'):
            return BOLD + CYAN + line + RESET
        return line


register('param', 'get', ParamGetColorizer())


class ParamDumpColorizer(Colorizer):
    """Colorizer for `ros2 param dump` — YAML parameter file."""

    def colorize(self, line: str) -> str:
        if line and not line.startswith(' ') and line.rstrip().endswith(':'):
            return BOLD + CYAN + line + RESET
        if line.startswith('  ') and line.rstrip().endswith(':'):
            return BOLD + GREEN + line + RESET
        if line.startswith('    ') and ':' in line:
            indent = len(line) - len(line.lstrip())
            rest = line.lstrip()
            key, _, val = rest.partition(':')
            return ' ' * indent + BLUE + key + RESET + ':' + CYAN + val + RESET
        return line


register('param', 'dump', ParamDumpColorizer())


class InterfaceShowColorizer(Colorizer):
    """Colorizer for `ros2 interface show` — IDL field definitions."""

    _RE_TYPE = re.compile(r'([\w]*\b(.+)/[\w]+)')
    _RE_BUILTIN = re.compile(
        r'\b(bool|byte|char|float32|float64|int8|int16|int32|int64'
        r'|uint8|uint16|uint32|uint64|string|wstring)\b'
    )

    def colorize(self, line: str) -> str:
        if line == '---':
            return DIM + '---' + RESET
        if line.lstrip().startswith('#'):
            return DIM + line + RESET

        # separate trailing inline comment before colorizing code
        inline_comment = ''
        comment_idx = line.find('#')
        if comment_idx != -1:
            inline_comment = DIM + line[comment_idx:] + RESET
            line = line[:comment_idx]

        if '=' in line:
            eq_idx = line.index('=')
            lhs = line[:eq_idx]
            rhs = line[eq_idx + 1:]
            lhs = self._RE_BUILTIN.sub(lambda m: GREEN + m.group(1) + RESET, lhs)
            return lhs + '=' + MAGENTA + rhs + RESET + inline_comment

        def _ct(m): return CYAN + m.group(1) + RESET
        line = self._RE_TYPE.sub(_ct, line)
        line = self._RE_BUILTIN.sub(lambda m: GREEN + m.group(1) + RESET, line)
        return line + inline_comment


register('interface', 'show', InterfaceShowColorizer())


class ComponentListColorizer(Colorizer):
    """Colorizer for `ros2 component list`."""

    def colorize(self, line: str) -> str:
        if line.startswith('/') and not line.startswith('  '):
            return BOLD + CYAN + line + RESET
        if line.startswith('  '):
            parts = line.split(None, 2)
            if len(parts) >= 2:
                uid = parts[0]
                name = parts[1] if len(parts) == 2 else ' '.join(parts[1:])
                return '  ' + DIM + uid + RESET + '  ' + YELLOW + name + RESET
        return line


register('component', 'list', ComponentListColorizer())
