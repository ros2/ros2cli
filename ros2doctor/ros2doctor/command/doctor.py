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

from ros2cli.command import CommandExtension
from ros2doctor.api import generate_report
from ros2doctor.api import run_checks
from ros2doctor.api.format import format_print


class DoctorCommand(CommandExtension):
    """Check ROS setup and other potential issues."""

    def add_arguments(self, parser, cli_name):
        group = parser.add_mutually_exclusive_group(required=False)
        group.add_argument(
            '--report', '-r', action='store_true',
            help='Print out all report info.'
        )
        group.add_argument(
            '--report-failed', '-rf', action='store_true',
            help='Print out report info on failed checks.'
        )

    def main(self, *, parser, args):
        """Run checks and print report to terminal based on user input args."""
        if args.report:
            cat_reports = generate_report()
            for _, report in cat_reports:
                format_print(report)
            return
        cat_results = run_checks()
        failed_cats = []
        failed = 0
        for cat, result in cat_results:
            if result is False:
                failed += 1
                failed_cats.append(cat)
        if failed != 0:
            print('\n%d/%d checks failed\n' % (failed, len(cat_results)))
            print('Failed tests are ', *failed_cats)
        else:
            print('\nAll %d checks passed\n' % len(cat_results))
        if args.report_failed and failed != 0:
            # need to run checks to get failed modules
            cat_reports = generate_report()
            for cat in failed_cats:
                for rcat, report in cat_reports:
                    if cat == rcat:
                        format_print(report)


class WtfCommand(DoctorCommand):
    """Use `wtf` as alias to `doctor`."""

    pass
