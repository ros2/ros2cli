# Copyright 2025 Open Source Robotics Foundation, Inc.
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

from typing import Iterable

from ros2doctor.api import Report


def generate_expected_topic_report(topic: str, pub_count: int, sub_count: int) -> Report:
    expected_report = Report('TOPIC LIST')
    expected_report.add_to_report('topic', topic)
    expected_report.add_to_report('publisher count', pub_count)
    expected_report.add_to_report('subscriber count', sub_count)
    return expected_report


def generate_expected_service_report(services: Iterable[str], serv_counts: Iterable[int],
                                     cli_counts: Iterable[int]) -> Report:
    expected_report = Report('SERVICE LIST')
    for service, serv_count, cli_count in zip(services, serv_counts, cli_counts):
        expected_report.add_to_report('service', service)
        expected_report.add_to_report('service count', serv_count)
        expected_report.add_to_report('client count', cli_count)
    return expected_report
