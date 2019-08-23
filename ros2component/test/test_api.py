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

import os

from domain_coordinator import get_coordinated_domain_id

from ros2cli.node.direct import DirectNode
from ros2cli.node.strategy import NodeStrategy

from ros2component.api import find_container_node_names
from ros2component.api import get_package_component_types

from ros2node.api import get_node_names


domain_id = get_coordinated_domain_id()  # Must keep this as a local to keep it alive
os.environ['ROS_DOMAIN_ID'] = str(domain_id)


def test_find_container_node_names():
    """Test find_container_node_names() API function."""
    with NodeStrategy([]) as node:
        node_names = get_node_names(node=node)

    with DirectNode([]) as node:
        assert len(find_container_node_names(
            node=node, node_names=node_names
        )) == 0

        assert len(find_container_node_names(
            node=node, node_names=[]
        )) == 0


def test_get_package_component_types():
    """Test get_package_component_types() API function."""
    assert len(get_package_component_types(package_name='ros2component')) == 0
