# Copyright 2017-2019 Open Source Robotics Foundation, Inc.
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

import argparse
from collections import namedtuple
import inspect
import os
from xmlrpc.server import SimpleXMLRPCRequestHandler
from xmlrpc.server import SimpleXMLRPCServer

import netifaces

import rclpy

from ros2cli.node.direct import DirectNode


def main(*, script_name='_ros2_daemon', argv=None):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--rmw-implementation', type=str, required=True,
        help='The RMW implementation name (must match the return value of '
             'rclpy.get_rmw_implementation_identifier())')
    parser.add_argument(
        '--ros-domain-id', metavar='N', type=int, required=True,
        help='The ROS domain id (must match the environment variable '
             'ROS_DOMAIN_ID)')
    parser.add_argument(
        '--timeout', metavar='N', type=int, default=2 * 60 * 60,
        help='Shutdown the daemon after N seconds of inactivity')
    args = parser.parse_args(args=argv)

    # the arguments are only passed for visibility in e.g. the process list
    assert args.rmw_implementation == rclpy.get_rmw_implementation_identifier()
    assert args.ros_domain_id == int(os.environ.get('ROS_DOMAIN_ID', 0))

    addr = ('localhost', get_daemon_port())
    NodeArgs = namedtuple(
        'NodeArgs', ('node_name_suffix', 'start_parameter_services'))
    node_args = NodeArgs(
        node_name_suffix='_daemon_%d' % args.ros_domain_id,
        start_parameter_services=False)
    with _DirectNode(node_args) as node:
        server = LocalXMLRPCServer(
            addr, logRequests=False, requestHandler=RequestHandler,
            allow_none=True)

        try:
            server.register_introspection_functions()

            # expose getter functions of node
            server.register_function(
                _print_invoked_function_name(
                    node.get_node_names_and_namespaces))
            server.register_function(
                _print_invoked_function_name(node.get_topic_names_and_types))
            server.register_function(
                _print_invoked_function_name(node.get_service_names_and_types))

            shutdown = False

            # shutdown the daemon in case of a timeout
            def timeout_handler():
                nonlocal shutdown
                print('Shutdown due to timeout')
                shutdown = True
            server.handle_timeout = timeout_handler
            server.timeout = args.timeout

            # function to shutdown daemon remotely
            def shutdown_handler():
                nonlocal shutdown
                print('Remote shutdown requested')
                shutdown = True
            server.register_function(shutdown_handler, 'system.shutdown')

            print('Serving XML-RPC on %s:%d/ros2cli/' % (addr[0], addr[1]))
            try:
                while not shutdown:
                    server.handle_request()
            except KeyboardInterrupt:
                pass
        finally:
            server.server_close()


class _DirectNode:
    """A direct node, that resets itself when a network interface changes."""

    def __init__(self, args):
        self.args = args
        # TODO(ivanpauno): A race condition is possible here, since it isn't possible to know
        # exactly which interfaces were available at node creation.
        self.node = DirectNode(args)
        self.addresses_at_start = self.interfaces_ip_addresses()

    def __enter__(self):
        self.node.__enter__()
        return self

    def __getattr__(self, name):
        attr = self.node.__getattr__(name)

        def wrapper():
            self.reset_if_addresses_changed()
            return self.node.__getattr__(name)()
        wrapper.__name__ = attr.__name__

        if inspect.ismethod(attr):
            return wrapper
        self.reset_if_addresses_changed()
        return attr

    def __exit__(self, exc_type, exc_value, traceback):
        self.node.__exit__(exc_type, exc_value, traceback)

    def interfaces_ip_addresses(self):
        addresses_by_interfaces = {}
        for (kind, info) in netifaces.gateways().items():
            if kind not in (netifaces.AF_INET, netifaces.AF_INET6):
                continue
            print('Interface kind: {}, info: {}'.format(kind, info))
            if isinstance(info, dict):
                continue
            info_list = info
            if isinstance(info, tuple):
                info_list = [info]
            addresses_by_interfaces[kind] = {}
            for item in info_list:
                # info[1] is the interface name
                addresses_by_interfaces[kind][item[1]] = netifaces.ifaddresses(item[1])[kind][0]['addr']
        print('Addresses by interfaces: {}'.format(addresses_by_interfaces))
        return addresses_by_interfaces

    def reset_if_addresses_changed(self):
        new_addresses = self.interfaces_ip_addresses()
        if new_addresses != self.addresses_at_start:
            self.addresses_at_start = new_addresses
            self.node.destroy_node()
            rclpy.shutdown()
            self.node = DirectNode(self.args)
            self.node.__enter__()
            print('Daemon node was reset')


class LocalXMLRPCServer(SimpleXMLRPCServer):

    def verify_request(self, request, client_address):
        if client_address[0] != '127.0.0.1':
            return False
        return super(LocalXMLRPCServer, self).verify_request(request, client_address)


def get_daemon_port():
    base_port = 11511
    base_port += int(os.environ.get('ROS_DOMAIN_ID', 0))
    return base_port


class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/ros2cli/',)


def _print_invoked_function_name(func):
    def wrapper():
        nonlocal func
        print('{func.__name__}()'.format_map(locals()))
        return func()
    wrapper.__name__ = func.__name__
    return wrapper


if __name__ == '__main__':
    main()
