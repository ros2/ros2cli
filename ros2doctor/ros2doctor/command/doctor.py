# Copyright 2019 Open Source Robotics Foundation, Inc.
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

from ros2cli.command import add_subparsers_on_demand
from ros2cli.command import CommandExtension
from ros2doctor.api import generate_reports
from ros2doctor.api import run_checks
from ros2doctor.api.format import format_print


class DoctorCommand(CommandExtension):
    """Check ROS setup and other potential issues."""

    def add_arguments(self, parser, cli_name):
        group = parser.add_mutually_exclusive_group(required=False)
        group.add_argument(
            '--report', '-r', action='store_true',
            help='Print all reports.'
        )
        group.add_argument(
            '--report-failed', '-rf', action='store_true',
            help='Print reports of failed checks only.'
        )
        parser.add_argument(
            '--exclude-packages', '-ep', action='store_true',
            help='Exclude package checks or report.'
        )
        parser.add_argument(
            '--include-warnings', '-iw', action='store_true',
            help='Include warnings as failed checks. Warnings are ignored by default.'
        )
        # add arguments and sub-commands of verbs
        add_subparsers_on_demand(
            parser, cli_name, '_verb', 'ros2doctor.verb', required=False)

    def main(self, *, parser, args):
        """Run checks and print report to terminal based on user input args."""
        if hasattr(args, '_verb'):
            extension = getattr(args, '_verb')
            return extension.main(args=args)

        # Local Variables to reduce code length
        iw, ep = (args.include_warnings, args.exclude_packages)
        # `ros2 doctor -r`
        if args.report:
            all_reports = generate_reports(exclude_packages=ep)
            for report_obj in all_reports:
                format_print(report_obj)
            return

        # `ros2 doctor

        fail_category, fail, total = run_checks(include_warnings=iw, exclude_packages=ep)
        if fail:
            print(f'\n{fail}/{total} check(s) failed\n')
            print('Failed modules:', *fail_category)
        else:
            print(f'\nAll {total} checks passed\n')

        # `ros2 doctor -rf`
        if args.report_failed and fail != 0:
            fail_reports = generate_reports(categories=fail_category)
            for report_obj in fail_reports:
                format_print(report_obj)


class WtfCommand(DoctorCommand):
    """Use `wtf` as alias to `doctor`."""

    pass
