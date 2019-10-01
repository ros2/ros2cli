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

from ros2doctor.api import Report
from ros2doctor.api.topic import TopicCheck
from ros2doctor.api.topic import TopicReport


def test_topic_check():
    topic_check = TopicCheck()
    check_result = topic_check.check()
    assert check_result.error == 0
    assert check_result.warning == 0


def _generate_expected_report(report_name, pub_count, sub_count):
    pass


def test_topic_report():
    
    pass
