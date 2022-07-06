# Copyright 2020 Open Source Robotics Foundation, Inc.
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

from xmlrpc.server import SimpleXMLRPCRequestHandler
from xmlrpc.server import SimpleXMLRPCServer

import netifaces


def get_local_ipaddrs():
    iplist = []
    interfaces = netifaces.interfaces()
    for interface in interfaces:
        addrs = netifaces.ifaddresses(interface)
        if netifaces.AF_INET in addrs.keys():
            for value in addrs[netifaces.AF_INET]:
                iplist.append(value['addr'])
    return iplist


class LocalXMLRPCServer(SimpleXMLRPCServer):

    def verify_request(self, request, client_address):
        if client_address[0] not in get_local_ipaddrs():
            return False
        return super(LocalXMLRPCServer, self).verify_request(request, client_address)


class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/ros2cli/',)
