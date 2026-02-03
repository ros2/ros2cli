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

"""Implementation of the 'ros2 wtf' command."""

from ros2cli.command import CommandExtension

from ros2wtf.api import (
    analyze_topic,
    analyze_node,
    analyze_system,
    format_issues,
    format_issues_json,
    format_issues_yaml,
    IssueSeverity,
)


class WtfCommand(CommandExtension):
    """Debug helper - What The Failure analysis for ROS 2 systems."""

    def add_arguments(self, parser, cli_name):
        """Add command-line arguments."""
        parser.add_argument(
            'target',
            nargs='?',
            default=None,
            help='Topic or node name to analyze (optional, analyzes entire system if not provided)'
        )
        parser.add_argument(
            '--topic', '-t',
            action='store_true',
            help='Treat target as a topic name'
        )
        parser.add_argument(
            '--node', '-n',
            action='store_true',
            help='Treat target as a node name'
        )
        parser.add_argument(
            '--all', '-a',
            action='store_true',
            help='Analyze entire system (default if no target specified)'
        )
        parser.add_argument(
            '--json', '-j',
            action='store_true',
            help='Output results in JSON format'
        )
        parser.add_argument(
            '--yaml', '-y',
            action='store_true',
            help='Output results in YAML format'
        )
        parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            help='Show detailed analysis information'
        )
        parser.add_argument(
            '--no-suggestions',
            action='store_true',
            help='Hide fix suggestions'
        )

    def main(self, *, args):
        """Execute the wtf command."""
        # Determine what to analyze
        if args.all or args.target is None:
            issues, info = analyze_system()
        elif args.topic:
            issues, info = analyze_topic(args.target)
        elif args.node:
            issues, info = analyze_node(args.target)
        else:
            # Auto-detect: try topic first, then node
            if args.target.startswith('/'):
                # Looks like a topic or fully qualified node name
                issues, info = analyze_topic(args.target)
                if not info.get('types') and not issues:
                    # Topic not found, try as node
                    issues, info = analyze_node(args.target)
            else:
                issues, info = analyze_system()

        # Format output
        if args.json:
            print(format_issues_json(issues, info))
        elif args.yaml:
            print(format_issues_yaml(issues, info))
        else:
            output = format_issues(
                issues,
                info,
                verbose=args.verbose,
                show_suggestions=not args.no_suggestions
            )
            print(output)

        # Return exit code based on issues found
        critical = len([i for i in issues if i.severity == IssueSeverity.CRITICAL])
        if critical > 0:
            return 3  # Critical issues
        elif issues:
            return 2  # Some issues found
        return 0  # No issues
