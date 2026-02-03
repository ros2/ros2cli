# Copyright 2024 Open Source Robotics Foundation, Inc.
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

"""Output formatters for ros2doctor command."""

import json
import sys
from datetime import datetime
from typing import Any, Dict, List

from .checks import CheckResult, CheckStatus


# ANSI color codes
class Colors:
    """ANSI color codes for terminal output."""

    PASS = '\033[92m'      # Green
    WARN = '\033[93m'      # Yellow
    FAIL = '\033[91m'      # Red
    INFO = '\033[94m'      # Blue
    BOLD = '\033[1m'       # Bold
    UNDERLINE = '\033[4m'  # Underline
    RESET = '\033[0m'      # Reset


def supports_color() -> bool:
    """Check if the terminal supports color output."""
    if not hasattr(sys.stdout, 'isatty'):
        return False
    if not sys.stdout.isatty():
        return False
    # Check for NO_COLOR environment variable
    import os
    if os.environ.get('NO_COLOR'):
        return False
    return True


def colorize(text: str, color: str, force: bool = False) -> str:
    """
    Apply color to text if terminal supports it.

    Args:
        text: Text to colorize
        color: Color name ('pass', 'warn', 'fail', 'info', 'bold')
        force: Force colorization even if terminal doesn't support it

    Returns:
        Colorized text string
    """
    if not force and not supports_color():
        return text

    color_map = {
        'pass': Colors.PASS,
        'green': Colors.PASS,
        'warn': Colors.WARN,
        'yellow': Colors.WARN,
        'fail': Colors.FAIL,
        'red': Colors.FAIL,
        'info': Colors.INFO,
        'blue': Colors.INFO,
        'bold': Colors.BOLD,
    }

    color_code = color_map.get(color.lower(), '')
    if color_code:
        return f'{color_code}{text}{Colors.RESET}'
    return text


def status_color(status: CheckStatus) -> str:
    """Get the color name for a check status."""
    color_map = {
        CheckStatus.PASS: 'pass',
        CheckStatus.WARN: 'warn',
        CheckStatus.FAIL: 'fail',
        CheckStatus.SKIP: 'info',
    }
    return color_map.get(status, 'info')


def format_results(
    results: List[CheckResult],
    summary: Dict[str, int],
    verbose: bool = False,
    show_failed_only: bool = False
) -> str:
    """
    Format check results for terminal output.

    Args:
        results: List of CheckResult objects
        summary: Summary statistics dict
        verbose: Show detailed information
        show_failed_only: Only show failed/warning checks

    Returns:
        Formatted string for terminal output
    """
    lines = []

    # Header
    header = 'ROS 2 System Health Report'
    separator = '=' * 80
    lines.append(colorize(separator, 'bold'))
    lines.append(colorize(header.center(80), 'bold'))
    lines.append(colorize(separator, 'bold'))
    lines.append('')

    # Results
    for result in results:
        if show_failed_only and result.status == CheckStatus.PASS:
            continue

        status_str = f'[{result.status.value}]'
        colored_status = colorize(status_str, status_color(result.status))

        lines.append(f'{colored_status} {result.name}')

        if result.message:
            lines.append(f'  {result.message}')

        if verbose and result.details:
            for key, value in result.details.items():
                if isinstance(value, list):
                    lines.append(f'  - {key}:')
                    for item in value[:5]:  # Limit to 5 items
                        lines.append(f'      {item}')
                    if len(value) > 5:
                        lines.append(f'      ... and {len(value) - 5} more')
                elif isinstance(value, dict):
                    lines.append(f'  - {key}:')
                    for k, v in value.items():
                        lines.append(f'      {k}: {v}')
                else:
                    lines.append(f'  - {key}: {value}')

        if result.suggestions:
            lines.append(colorize('  Suggestions:', 'info'))
            for suggestion in result.suggestions:
                lines.append(f'    - {suggestion}')

        lines.append('')

    # Summary
    lines.append('-' * 80)

    passed = colorize(f'{summary["passed"]} passed', 'pass')
    warnings = colorize(f'{summary["warnings"]} warnings', 'warn')
    failed = colorize(f'{summary["failed"]} failed', 'fail')

    summary_line = f'Summary: {passed}, {warnings}, {failed}'
    if summary.get('skipped', 0) > 0:
        skipped = colorize(f'{summary["skipped"]} skipped', 'info')
        summary_line += f', {skipped}'

    lines.append(summary_line)
    lines.append('-' * 80)

    return '\n'.join(lines)


def format_json(
    results: List[CheckResult],
    summary: Dict[str, int],
    include_timestamp: bool = True
) -> str:
    """
    Format check results as JSON.

    Args:
        results: List of CheckResult objects
        summary: Summary statistics dict
        include_timestamp: Include timestamp in output

    Returns:
        JSON formatted string
    """
    output: Dict[str, Any] = {
        'command': 'ros2 doctor',
        'summary': summary,
        'checks': []
    }

    if include_timestamp:
        output['timestamp'] = datetime.utcnow().isoformat() + 'Z'

    for result in results:
        check_data: Dict[str, Any] = {
            'name': result.name,
            'status': result.status.value,
        }
        if result.message:
            check_data['message'] = result.message
        if result.details:
            check_data['details'] = result.details
        if result.suggestions:
            check_data['suggestions'] = result.suggestions

        output['checks'].append(check_data)

    return json.dumps(output, indent=2, default=str)


def format_yaml(
    results: List[CheckResult],
    summary: Dict[str, int],
    include_timestamp: bool = True
) -> str:
    """
    Format check results as YAML.

    Args:
        results: List of CheckResult objects
        summary: Summary statistics dict
        include_timestamp: Include timestamp in output

    Returns:
        YAML formatted string
    """
    lines = ['command: ros2 doctor']

    if include_timestamp:
        lines.append(f'timestamp: {datetime.utcnow().isoformat()}Z')

    lines.append('summary:')
    lines.append(f'  passed: {summary["passed"]}')
    lines.append(f'  warnings: {summary["warnings"]}')
    lines.append(f'  failed: {summary["failed"]}')
    if summary.get('skipped', 0) > 0:
        lines.append(f'  skipped: {summary["skipped"]}')

    lines.append('checks:')

    for result in results:
        lines.append(f'  - name: {result.name}')
        lines.append(f'    status: {result.status.value}')

        if result.message:
            # Escape quotes in message
            msg = result.message.replace('"', '\\"')
            lines.append(f'    message: "{msg}"')

        if result.details:
            lines.append('    details:')
            for key, value in result.details.items():
                if isinstance(value, list):
                    lines.append(f'      {key}:')
                    for item in value:
                        lines.append(f'        - {item}')
                elif isinstance(value, dict):
                    lines.append(f'      {key}:')
                    for k, v in value.items():
                        lines.append(f'        {k}: {v}')
                elif isinstance(value, bool):
                    lines.append(f'      {key}: {str(value).lower()}')
                else:
                    lines.append(f'      {key}: {value}')

        if result.suggestions:
            lines.append('    suggestions:')
            for suggestion in result.suggestions:
                lines.append(f'      - "{suggestion}"')

    return '\n'.join(lines)


def print_report_notice():
    """Print notice about sensitive information in reports."""
    print()
    print(colorize('Note:', 'bold'), end=' ')
    print('The report above may contain sensitive information such as')
    print('environment variables, file paths, and system configuration.')
    print('Please review before sharing.')
