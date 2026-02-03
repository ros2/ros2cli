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

"""Output formatters for ros2wtf command."""

import json
import sys
from datetime import datetime
from typing import Any, Dict, List

from .analyzer import Issue, IssueSeverity


# ANSI color codes
class Colors:
    """ANSI color codes for terminal output."""
    RED = '\033[91m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    BLUE = '\033[94m'
    MAGENTA = '\033[95m'
    CYAN = '\033[96m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    RESET = '\033[0m'


def supports_color() -> bool:
    """Check if the terminal supports color output."""
    import os
    if not hasattr(sys.stdout, 'isatty'):
        return False
    if not sys.stdout.isatty():
        return False
    if os.environ.get('NO_COLOR'):
        return False
    return True


def colorize(text: str, color: str, force: bool = False) -> str:
    """
    Apply color to text if terminal supports it.

    Args:
        text: Text to colorize
        color: Color name
        force: Force colorization

    Returns:
        Colorized text string
    """
    if not force and not supports_color():
        return text

    color_map = {
        'red': Colors.RED,
        'green': Colors.GREEN,
        'yellow': Colors.YELLOW,
        'blue': Colors.BLUE,
        'magenta': Colors.MAGENTA,
        'cyan': Colors.CYAN,
        'bold': Colors.BOLD,
        'underline': Colors.UNDERLINE,
    }

    color_code = color_map.get(color.lower(), '')
    if color_code:
        return f'{color_code}{text}{Colors.RESET}'
    return text


def severity_color(severity: IssueSeverity) -> str:
    """Get the color name for an issue severity."""
    color_map = {
        IssueSeverity.CRITICAL: 'red',
        IssueSeverity.WARNING: 'yellow',
        IssueSeverity.INFO: 'blue',
    }
    return color_map.get(severity, 'blue')


def format_issues(
    issues: List[Issue],
    info: Dict[str, Any],
    verbose: bool = False,
    show_suggestions: bool = True
) -> str:
    """
    Format issues for terminal output.

    Args:
        issues: List of Issue objects
        info: Additional info dict
        verbose: Show detailed information
        show_suggestions: Show fix suggestions

    Returns:
        Formatted string for terminal output
    """
    lines = []
    separator = '=' * 80

    # Header
    lines.append(colorize(separator, 'bold'))
    lines.append(colorize('WTF Analysis Report'.center(80), 'bold'))
    lines.append(colorize(separator, 'bold'))
    lines.append('')

    if not issues:
        lines.append(colorize('No issues detected!', 'green'))
        lines.append('')
        if info:
            lines.append('System Info:')
            for key, value in info.items():
                if not isinstance(value, list) or verbose:
                    lines.append(f'  {key}: {value}')
        lines.append('')
        lines.append(colorize(separator, 'bold'))
        return '\n'.join(lines)

    # Issues
    for i, issue in enumerate(issues, 1):
        severity_str = issue.severity.value.upper()
        colored_severity = colorize(severity_str, severity_color(issue.severity))

        lines.append(f'ISSUE #{i}: {issue.title} [{colored_severity}]')
        lines.append('-' * 80)

        lines.append(colorize('Problem:', 'bold'))
        for line in issue.problem.split('\n'):
            lines.append(f'  {line}')
        lines.append('')

        lines.append(colorize('Impact:', 'bold'))
        lines.append(f'  {issue.impact}')
        lines.append('')

        if show_suggestions and issue.suggestions:
            lines.append(colorize('Suggestions:', 'cyan'))
            for j, suggestion in enumerate(issue.suggestions, 1):
                lines.append(f'  Option {chr(64 + j)}: {suggestion}')
            lines.append('')

        if verbose and issue.details:
            lines.append(colorize('Details:', 'bold'))
            for key, value in issue.details.items():
                lines.append(f'  {key}: {value}')
            lines.append('')

        lines.append(separator)
        lines.append('')

    # Summary
    critical = len([i for i in issues if i.severity == IssueSeverity.CRITICAL])
    warnings = len([i for i in issues if i.severity == IssueSeverity.WARNING])

    summary_parts = [f'Found {len(issues)} issue(s)']
    if critical:
        summary_parts.append(colorize(f'{critical} critical', 'red'))
    if warnings:
        summary_parts.append(colorize(f'{warnings} warning(s)', 'yellow'))

    lines.append(' | '.join(summary_parts))
    lines.append(separator)

    return '\n'.join(lines)


def format_issues_json(
    issues: List[Issue],
    info: Dict[str, Any],
    include_timestamp: bool = True
) -> str:
    """
    Format issues as JSON.

    Args:
        issues: List of Issue objects
        info: Additional info dict
        include_timestamp: Include timestamp in output

    Returns:
        JSON formatted string
    """
    output: Dict[str, Any] = {
        'command': 'ros2 wtf',
        'summary': {
            'total': len(issues),
            'critical': len([i for i in issues if i.severity == IssueSeverity.CRITICAL]),
            'warnings': len([i for i in issues if i.severity == IssueSeverity.WARNING]),
            'info': len([i for i in issues if i.severity == IssueSeverity.INFO]),
        },
        'info': info,
        'issues': []
    }

    if include_timestamp:
        output['timestamp'] = datetime.utcnow().isoformat() + 'Z'

    for issue in issues:
        issue_data: Dict[str, Any] = {
            'title': issue.title,
            'severity': issue.severity.value,
            'problem': issue.problem,
            'impact': issue.impact,
        }
        if issue.topic:
            issue_data['topic'] = issue.topic
        if issue.node:
            issue_data['node'] = issue.node
        if issue.suggestions:
            issue_data['suggestions'] = issue.suggestions
        if issue.details:
            issue_data['details'] = issue.details

        output['issues'].append(issue_data)

    return json.dumps(output, indent=2, default=str)


def format_issues_yaml(
    issues: List[Issue],
    info: Dict[str, Any],
    include_timestamp: bool = True
) -> str:
    """
    Format issues as YAML.

    Args:
        issues: List of Issue objects
        info: Additional info dict
        include_timestamp: Include timestamp in output

    Returns:
        YAML formatted string
    """
    lines = ['command: ros2 wtf']

    if include_timestamp:
        lines.append(f'timestamp: {datetime.utcnow().isoformat()}Z')

    # Summary
    lines.append('summary:')
    lines.append(f'  total: {len(issues)}')
    lines.append(f'  critical: {len([i for i in issues if i.severity == IssueSeverity.CRITICAL])}')
    lines.append(f'  warnings: {len([i for i in issues if i.severity == IssueSeverity.WARNING])}')
    lines.append(f'  info: {len([i for i in issues if i.severity == IssueSeverity.INFO])}')

    # Info
    if info:
        lines.append('info:')
        for key, value in info.items():
            if isinstance(value, list):
                lines.append(f'  {key}:')
                for item in value:
                    lines.append(f'    - {item}')
            else:
                lines.append(f'  {key}: {value}')

    # Issues
    lines.append('issues:')

    for issue in issues:
        lines.append(f'  - title: "{issue.title}"')
        lines.append(f'    severity: {issue.severity.value}')

        # Multi-line problem
        problem_escaped = issue.problem.replace('"', '\\"').replace('\n', '\\n')
        lines.append(f'    problem: "{problem_escaped}"')

        lines.append(f'    impact: "{issue.impact}"')

        if issue.topic:
            lines.append(f'    topic: {issue.topic}')
        if issue.node:
            lines.append(f'    node: {issue.node}')

        if issue.suggestions:
            lines.append('    suggestions:')
            for suggestion in issue.suggestions:
                suggestion_escaped = suggestion.replace('"', '\\"').replace('\n', ' ')
                lines.append(f'      - "{suggestion_escaped}"')

        if issue.details:
            lines.append('    details:')
            for key, value in issue.details.items():
                if isinstance(value, list):
                    lines.append(f'      {key}:')
                    for item in value:
                        lines.append(f'        - {item}')
                else:
                    lines.append(f'      {key}: {value}')

    return '\n'.join(lines)
