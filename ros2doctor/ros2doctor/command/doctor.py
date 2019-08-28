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
from ros2doctor.api.format import format_print
from ros2doctor.api import generate_report
from ros2doctor.api import run_checks


class DoctorCommand(CommandExtension):
    """Check ROS setup and other potential issues."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument(
            '--report', '-r', action='store_true',
            help='Print out all report info.'
        )
        parser.add_argument(
            '--report_failed', '-rf', action='store_true',
            help="Print out report info on failed checks."
        )

    def main(self, *, parser, args):
        if args.report:
            report = generate_report()
            format_print(report.keys(), report)
            return 
        check_results, failed_checks, failed_modules = run_checks()
        failed = check_results.count(False)
        passed = check_results.count(True)
        if failed != 0:
            print('%d/%d checks failed' % (failed, len(check_results)))
            print('Failed checks:', *failed_checks)
        else:
            print('%d/%d checks passed' % (passed, len(check_results)))
        if args.report_failed:
            report = generate_report()
            format_print(failed_modules, report)


class WtfCommand(DoctorCommand):
    """Use `wtf` as alias to `doctor`."""

    pass
