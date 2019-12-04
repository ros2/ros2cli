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

from ros2cli.command import add_subparsers
from ros2cli.command import CommandExtension
from ros2cli.verb import get_verb_extensions
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
            '--include-warnings', '-iw', action='store_true',
            help='Include warnings as failed checks. Warnings are ignored by default.'
        )

        verb_extensions = get_verb_extensions('ros2doctor.verb')
        add_subparsers(
            parser, cli_name, '_verb', verb_extensions, required=False)

    def main(self, *, parser, args):
        """Run checks and print report to terminal based on user input args."""
        if hasattr(args, '_verb'):
            extension = getattr(args, '_verb')
            return extension.main(args=args)
        
        # `ros2 doctor -r`
        if args.report:
            all_reports = generate_reports()
            for report_obj in all_reports:
                format_print(report_obj)
            return

        # `ros2 doctor`
        failed_cats, fail, total = run_checks(include_warnings=args.include_warnings)
        if fail:
            print('\n%d/%d checks failed\n' % (fail, total))
            print('Failed modules:', *failed_cats)
        else:
            print('\nAll %d checks passed\n' % total)

        # `ros2 doctor -rf`
        if args.report_failed and fail != 0:
            fail_reports = generate_reports(categories=failed_cats)
            for report_obj in fail_reports:
                format_print(report_obj)


class WtfCommand(DoctorCommand):
    """Use `wtf` as alias to `doctor`."""

    pass
