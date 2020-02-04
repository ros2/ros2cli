# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.srv import GetAvailableTransitions
from lifecycle_msgs.srv import GetState

import rclpy

from ros2node.api import get_node_names as get_all_node_names

from ros2service.api import get_service_names_and_types


def get_node_names(*, node, include_hidden_nodes=False):
    node_names = get_all_node_names(
        node=node, include_hidden_nodes=include_hidden_nodes)
    service_names_and_types = get_service_names_and_types(
        node=node, include_hidden_services=include_hidden_nodes)
    return [
        n for n in node_names
        if _has_lifecycle(n.full_name, service_names_and_types)]


def _has_lifecycle(node_name, service_names_and_types):
    for (service_name, service_types) in service_names_and_types:
        if (
            service_name == f'{node_name}/get_state' and
            'lifecycle_msgs/srv/GetState' in service_types
        ):
            return True
    return False


def call_get_states(*, node, node_names):
    clients = {}
    futures = {}
    # create clients
    for node_name in node_names:
        clients[node_name] = \
            node.create_client(GetState, f'{node_name}/get_state')

    # wait until all clients have been called
    while True:
        for node_name in [n for n in node_names if n not in futures]:
            # call as soon as ready
            client = clients[node_name]
            if client.service_is_ready():
                request = GetState.Request()
                future = client.call_async(request)
                futures[node_name] = future

        if len(futures) == len(clients):
            break
        rclpy.spin_once(node, timeout_sec=1.0)

    # wait for all responses
    for future in futures.values():
        rclpy.spin_until_future_complete(node, future)

    # return current state or exception for each node
    states = {}
    for node_name, future in futures.items():
        if future.result() is not None:
            response = future.result()
            states[node_name] = response.current_state
        else:
            states[node_name] = future.exception()
    return states


def call_get_available_transitions(*, node, states):
    return _call_get_transitions(node, states, 'get_available_transitions')


def call_get_transition_graph(*, node, states):
    return _call_get_transitions(node, states, 'get_transition_graph')


def _call_get_transitions(node, states, service_name):
    clients = {}
    futures = {}
    # create clients
    for node_name in states.keys():
        clients[node_name] = node.create_client(
            GetAvailableTransitions, f'{node_name}/{service_name}')

    # wait until all clients have been called
    while True:
        for node_name in [n for n in states.keys() if n not in futures]:
            # call as soon as ready
            client = clients[node_name]
            if client.service_is_ready():
                request = GetAvailableTransitions.Request()
                future = client.call_async(request)
                futures[node_name] = future

        if len(futures) == len(clients):
            break
        rclpy.spin_once(node, timeout_sec=1.0)

    # wait for all responses
    for future in futures.values():
        rclpy.spin_until_future_complete(node, future)

    # return transitions from current state or exception for each node
    transitions = {}
    for node_name, future in futures.items():
        if future.result() is not None:
            response = future.result()
            transitions[node_name] = []
            for transition_description in response.available_transitions:
                if (
                    states[node_name] is None or
                    transition_description.start_state == states[node_name]
                ):
                    transitions[node_name].append(
                        transition_description)
        else:
            transitions[node_name] = future.exception()
    return transitions


def call_change_states(*, node, transitions):
    clients = {}
    futures = {}
    # create clients
    for node_name in transitions.keys():
        clients[node_name] = node.create_client(
            ChangeState, f'{node_name}/change_state')

    # wait until all clients have been called
    while True:
        for node_name in [n for n in transitions.keys() if n not in futures]:
            # call as soon as ready
            client = clients[node_name]
            if client.service_is_ready():
                request = ChangeState.Request()
                request.transition = transitions[node_name]
                future = client.call_async(request)
                futures[node_name] = future

        if len(futures) == len(clients):
            break
        rclpy.spin_once(node, timeout_sec=1.0)

    # wait for all responses
    for future in futures.values():
        rclpy.spin_until_future_complete(node, future)

    # return success flag or exception for each node
    results = {}
    for node_name, future in futures.items():
        if future.result() is not None:
            response = future.result()
            results[node_name] = response.success
        else:
            results[node_name] = future.exception()
    return results
