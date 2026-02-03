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

"""Verb implementation for 'ros2 wtf' command."""

from ros2cli.verb import VerbExtension

from ros2wtf.api import (
    analyze_topic,
    analyze_node,
    analyze_system,
    format_issues,
    format_issues_json,
    format_issues_yaml,
    IssueSeverity,
)


class WtfVerb(VerbExtension):
    """Debug helper - What The Failure analysis for ROS 2 systems."""

    def add_arguments(self, parser, cli_name):
        """Add command-line arguments for the wtf verb."""
        parser.add_argument(
            'target',
            nargs='?',
            default=None,
            help='Topic or node name to analyze (optional)'
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
            help='Analyze entire system'
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
        parser.add_argument(
            '--watch', '-w',
            action='store_true',
            help='Continuous monitoring mode (analyze periodically)'
        )
        parser.add_argument(
            '--interval', '-i',
            type=float,
            default=5.0,
            help='Update interval in seconds for watch mode (default: 5.0)'
        )

    def main(self, *, args):
        """Execute the wtf verb."""
        import time
        import sys

        while True:
            # Clear screen in watch mode
            if args.watch:
                print('\033[2J\033[H', end='')

            # Determine what to analyze
            if args.all or args.target is None:
                issues, info = analyze_system()
            elif args.topic:
                issues, info = analyze_topic(args.target)
            elif args.node:
                issues, info = analyze_node(args.target)
            else:
                # Auto-detect
                if args.target.startswith('/'):
                    issues, info = analyze_topic(args.target)
                    if not info.get('types') and not issues:
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

            # Exit if not in watch mode
            if not args.watch:
                break

            # Wait for next iteration
            try:
                time.sleep(args.interval)
            except KeyboardInterrupt:
                print('\nWatch mode stopped.')
                break

        # Return exit code
        critical = len([i for i in issues if i.severity == IssueSeverity.CRITICAL])
        if critical > 0:
            return 3
        elif issues:
            return 2
        return 0
