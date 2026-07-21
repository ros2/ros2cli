# Copyright 2026 Tomoya Fujita, Fumiya Ohnishi
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

from rcl_interfaces.msg import LoggerLevel

from ros2log.verb import VerbExtension

# Mapping from LoggerLevel constant names (LOG_LEVEL_*) to display names.
# LOG_LEVEL_UNKNOWN is displayed as UNSET to match rcutils convention.
_LOGGER_LEVEL_DISPLAY_NAMES = {
    'LOG_LEVEL_UNKNOWN': 'UNSET',
    'LOG_LEVEL_DEBUG': 'DEBUG',
    'LOG_LEVEL_INFO': 'INFO',
    'LOG_LEVEL_WARN': 'WARN',
    'LOG_LEVEL_ERROR': 'ERROR',
    'LOG_LEVEL_FATAL': 'FATAL',
}

# Descriptions for each log level.
LOG_LEVEL_DESCRIPTIONS = {
    'UNSET': (
        'The logger level is not set explicitly and will use '
        'the default or inherited level.'
    ),
    'DEBUG': (
        'Debug is for pedantic information, which is useful '
        'when debugging issues.'
    ),
    'INFO': (
        'Info is the standard informational level and is used '
        'to report expected information.'
    ),
    'WARN': (
        'Warning is for information that may potentially cause '
        'issues or possibly unexpected behavior.'
    ),
    'ERROR': (
        'Error is for information that this node cannot resolve.'
    ),
    'FATAL': (
        'Information about an impending node shutdown.'
    ),
}


def get_log_levels_from_msg() -> list[tuple[str, int]]:
    """
    Extract log level constants from rcl_interfaces.msg.LoggerLevel.

    Returns a sorted list of (name, value) tuples, where name is the
    display name (e.g. 'DEBUG') rather than the constant name (e.g.
    'LOG_LEVEL_DEBUG').
    """
    levels = []
    for attr in dir(LoggerLevel):
        if attr not in _LOGGER_LEVEL_DISPLAY_NAMES:
            continue
        val = getattr(LoggerLevel, attr)
        display_name = _LOGGER_LEVEL_DISPLAY_NAMES[attr]
        levels.append((display_name, val))

    levels.sort(key=lambda x: x[1])
    return levels


# Build the level map once at import time for reuse by other verbs (get, set).
_LOG_LEVELS = get_log_levels_from_msg()
LEVEL_NAME_TO_VALUE = dict(_LOG_LEVELS)
LEVEL_VALUE_TO_NAME = {value: name for name, value in _LOG_LEVELS}


class LevelsVerb(VerbExtension):
    """Show all valid log level values."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--value', '-v',
            action='store_true',
            default=False,
            help='Show numeric values alongside level names')

    def main(self, *, args):
        levels = get_log_levels_from_msg()
        for name, value in levels:
            desc = LOG_LEVEL_DESCRIPTIONS.get(name, '')
            if args.value:
                print(f'{name:<7} ({value:>2}) : {desc}')
            else:
                print(f'{name:<7} : {desc}')
        return 0
