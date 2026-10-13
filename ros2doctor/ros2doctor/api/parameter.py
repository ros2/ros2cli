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

import rclpy
from rclpy.parameter import parameter_value_to_python
from ros2param.api import call_get_parameters
from ros2param.api import call_list_parameters

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy
from ros2doctor.api import DoctorCheck
from ros2doctor.api import DoctorReport
from ros2doctor.api import Report
from ros2doctor.api import Result
from ros2doctor.api.format import doctor_warn


class ParameterReport(DoctorReport):
    """Report parameter related information."""

    def category(self):
        return 'parameter'

    def report(self):
        report = Report('PARAMETER LIST')
        with NodeStrategy(None) as node:
            try:
                node_names_and_namespaces = \
                    node.get_node_names_and_namespaces()
            except Exception:
                node_names_and_namespaces = []
            if not node_names_and_namespaces:
                report.add_to_report('total nodes checked', 0)
                report.add_to_report('total parameter count', 0)
                report.add_to_report('parameter', 'none')
                return report

            param_count = 0
            nodes_checked = 0

            with DirectNode(None) as param_node:
                for node_name, namespace in sorted(node_names_and_namespaces):
                    nodes_checked += 1
                    full_name = f"{namespace.rstrip('/')}/{node_name}"
                    try:
                        response = call_list_parameters(
                            node=param_node.node, node_name=full_name)
                        if response is None:
                            continue
                        elif response.result() is None:
                            continue

                        param_names = response.result().result.names
                        if param_names:
                            param_count += len(param_names)
                            report.add_to_report('node', full_name)
                            try:
                                param_response = call_get_parameters(
                                    node=param_node.node,
                                    node_name=full_name,
                                    parameter_names=param_names
                                )
                                param_values = None
                                if param_response:
                                    param_values = param_response.values
                                if param_values and len(param_values) == len(
                                    param_names
                                ):
                                    params_with_values = sorted(
                                        zip(param_names, param_values)
                                    )
                                    for name, value_msg in params_with_values:
                                        value = parameter_value_to_python(
                                            value_msg
                                        )
                                        report.add_to_report(
                                            'parameter', f'{name}: {value}')
                                else:
                                    for param_name in sorted(param_names):
                                        report.add_to_report(
                                            'parameter', param_name
                                        )
                            except RuntimeError:
                                for param_name in sorted(param_names):
                                    report.add_to_report(
                                        'parameter', param_name
                                    )
                    except RuntimeError:
                        pass

            report.add_to_report('total nodes checked', nodes_checked)
            report.add_to_report('total parameter count', param_count)
        return report
