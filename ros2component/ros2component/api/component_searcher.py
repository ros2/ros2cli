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

class ComponentSearcher:
    """An abstract class providing interfaces to get component types"""

    def __init__(self, component_resource_type):
        self.component_resource_type = component_resource_type

    def get_component_types(self):
        """Get the registered component types
        :return: a list of component types in the tuple(res type/package name, component name)
        """
        pass

    def get_package_component_types(self, package_name):
        """Get the registered component types in a package
        :param package_name: whose component types are to be retrieved.
        :return: a list of component types in the tuple(res type/package name, component name)
        """
        pass
