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
        all_result, failed_names = run_checks()
        failed = all_result.count(False)
        passed = all_result.count(True)
        if failed != 0:
            print('%d/%d checks failed' % (failed, len(all_result)))
            print('Failed checks:', *failed_names)
        else:
            print('%d/%d checks passed' % (passed, len(all_result)))

        if args.report:
            all_report = generate_report()
            format_print(all_names, all_report)
            # for name, module_report in all_report.items():
            #     for k, v in module_report.items():
            #         print_term(k, v)
        if args.report_failed:
            format_print(failed_names, all_report)
            # for module_name in failed_names:
            #     module_report = all_report.get(module_name)
            #     if module_report:
            #         for k, v in module_report:
            #             print_term(k, v)


class WtfCommand(DoctorCommand):
    """Use `wtf` as alias to `doctor`."""

    pass
