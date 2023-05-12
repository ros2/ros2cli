# Copyright 2023 Open Source Robotics Foundation, Inc.
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
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState

from test_msgs.srv import BasicTypes


class IntrospectableService(Node):

    def __init__(self):
        super().__init__('introspectable_service')
        self.service = self.create_service(BasicTypes, 'test_introspectable', self.callback)
        self.service.configure_introspection(
            self.get_clock(), qos_profile_system_default, ServiceIntrospectionState.CONTENTS)

    def callback(self, request, response):
        for field_name in request.get_fields_and_field_types():
            setattr(response, field_name, getattr(request, field_name))
        return response


class IntrospectableClient(Node):

    def __init__(self):
        super().__init__('introspectable_client')
        self.client = self.create_client(BasicTypes, 'test_introspectable')
        self.client.configure_introspection(
            self.get_clock(), qos_profile_system_default, ServiceIntrospectionState.CONTENTS)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.future = None

    def timer_callback(self):
        if not self.client.service_is_ready():
            return

        if self.future is None:
            request = BasicTypes.Request()
            request.bool_value = True
            request.int32_value = 42
            request.string_value = 'test_string_value'
            self.future = self.client.call_async(request)
            return

        if not self.future.done():
            return

        if self.future.result() is None:
            self.get_logger().error(f'Exception calling service: {self.future.exception()!r}')

        self.future = None


def main(args=None):
    rclpy.init(args=args)

    service_node = IntrospectableService()
    client_node = IntrospectableClient()

    executor = SingleThreadedExecutor()
    executor.add_node(service_node)
    executor.add_node(client_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        executor.remove_node(client_node)
        executor.remove_node(service_node)
        executor.shutdown()
        service_node.destroy_node()
        client_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
