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

"""Verb implementation for 'ros2 doctor' command."""

from ros2cli.verb import VerbExtension

from ros2doctor.api import (
    run_all_checks,
    format_results,
    format_json,
    format_yaml,
    print_report_notice,
)


class DoctorVerb(VerbExtension):
    """Check ROS 2 setup and diagnose potential issues."""

    def add_arguments(self, parser, cli_name):
        """Add command-line arguments for the doctor verb."""
        parser.add_argument(
            '--report', '-r',
            action='store_true',
            help='Print full diagnostic report with details'
        )
        parser.add_argument(
            '--report-failed', '-rf',
            action='store_true',
            help='Print report only for failed and warning checks'
        )
        parser.add_argument(
            '--include-warnings', '-iw',
            action='store_true',
            default=False,
            help='Include warnings when counting failures (for exit code)'
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
            help='Show detailed diagnostic information'
        )
        parser.add_argument(
            '--summary', '-s',
            action='store_true',
            help='Show only summary (suppress individual check output)'
        )
        parser.add_argument(
            '--exclude',
            nargs='*',
            default=[],
            metavar='CHECK',
            help='Exclude specific checks (e.g., --exclude "Resource Usage")'
        )
        parser.add_argument(
            '--check',
            choices=['rmw', 'nodes', 'qos', 'resources', 'environment', 'all'],
            default='all',
            help='Run specific check only'
        )

    def main(self, *, args):
        """Execute the doctor verb."""
        # Map check argument to exclude list
        check_mapping = {
            'rmw': ['Node Connectivity', 'QoS Compatibility', 'Resource Usage', 'Environment'],
            'nodes': ['RMW Configuration', 'QoS Compatibility', 'Resource Usage', 'Environment'],
            'qos': ['RMW Configuration', 'Node Connectivity', 'Resource Usage', 'Environment'],
            'resources': ['RMW Configuration', 'Node Connectivity', 'QoS Compatibility', 'Environment'],
            'environment': ['RMW Configuration', 'Node Connectivity', 'QoS Compatibility', 'Resource Usage'],
            'all': [],
        }

        exclude_checks = list(args.exclude)
        if args.check != 'all':
            exclude_checks.extend(check_mapping.get(args.check, []))

        # Run checks
        results, summary = run_all_checks(
            include_warnings=args.include_warnings,
            exclude_checks=exclude_checks
        )

        # Determine output format
        if args.json:
            print(format_json(results, summary))
        elif args.yaml:
            print(format_yaml(results, summary))
        elif args.summary:
            # Just print summary line
            total = summary['passed'] + summary['warnings'] + summary['failed']
            print(f"Ran {total} checks: "
                  f"{summary['passed']} passed, "
                  f"{summary['warnings']} warnings, "
                  f"{summary['failed']} failed")
        else:
            # Terminal output
            verbose = args.report or args.verbose
            show_failed = args.report_failed

            output = format_results(
                results,
                summary,
                verbose=verbose,
                show_failed_only=show_failed
            )
            print(output)

            # Print notice if showing full report
            if verbose:
                print_report_notice()

        # Calculate exit code
        if args.include_warnings:
            failures = summary['failed'] + summary['warnings']
        else:
            failures = summary['failed']

        if failures > 0:
            return 2
        return 0
