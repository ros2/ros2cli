# Copyright 2017 Open Source Robotics Foundation, Inc.
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
import os

from collections import namedtuple

from ros2cli.node.direct import DirectNode
from xmlrpc.server import SimpleXMLRPCServer


def main(*, script_name='_ros2_daemon', argv=None):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--timeout', metavar='N', type=int, default=2 * 60 * 60,
        help='Shutdown the daemin after N seconds of inactivity')
    args = parser.parse_args(args=argv)

    addr = ('localhost', get_daemon_port())
    NodeArgs = namedtuple('NodeArgs', 'node_name_suffix')
    node_args = NodeArgs(node_name_suffix='_daemon')
    with DirectNode(node_args) as node:
        server = SimpleXMLRPCServer(addr, logRequests=False)

        try:
            server.register_introspection_functions()

            # expose getter functions of node
            server.register_function(
                _print_invoked_function_name(node.get_node_names))
            server.register_function(
                _print_invoked_function_name(node.get_topic_names_and_types))

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

            print('Serving XML-RPC on %s:%d' % addr)
            try:
                while not shutdown:
                    server.handle_request()
            except KeyboardInterrupt:
                pass
        finally:
            server.server_close()


def get_daemon_port():
    base_port = 11511
    base_port += int(os.environ.get('ROS_DOMAIN_ID', 0))
    return base_port


def _print_invoked_function_name(func):
    def wrapper():
        nonlocal func
        print('{func.__name__}()'.format_map(locals()))
        return func()
    wrapper.__name__ = func.__name__
    return wrapper
