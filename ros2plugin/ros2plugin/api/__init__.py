# Copyright 2019 Canonical Ltd.
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

from ament_index_python.resources import get_resource
from ament_index_python.resources import get_resource_types
from ament_index_python.resources import get_resources
from ament_index_python.resources import has_resource

PLUGIN_RESOURCE_TYPE = '__pluginlib__plugin'


def is_plugin_ressource_type(resource_type):
    """
    Check if resource_type has the plugin extension.

    :param str resource_type: the ressource type name to be evaluated.

    :return: a boolean, True if resource_type has the plugin extension.
    """
    assert resource_type, 'The resource type must not be empty'
    return PLUGIN_RESOURCE_TYPE in resource_type


def get_registered_plugin_ressource_list():
    """
    Get all plugin ressources registered in the ament index.

    :return: a filtered list containing the plugin ressouce types.
    """
    return filter(is_plugin_ressource_type, get_resource_types())


def get_package_names_with_plugin_ressource_types():
    """
    Get the names of all packages that register a plugin ressource in the ament index.

    :return: a list of packages exporting plugins.
    """
    packages = []
    for plugin in get_registered_plugin_ressource_list():
        packages += list(get_resources(plugin).keys())
    return packages


def get_package_plugin_ressource(*, package_name=None):
    """
    Get all plugin ressources registered in the ament index for the given package.

    :param package_name: whose component types are to be retrieved.
    :return: a list of plugin ressources relative path.
    """
    plugin_ressources = get_registered_plugin_ressource_list()
    package_plugins = []
    for plugin_ressource in plugin_ressources:
        if has_resource(plugin_ressource, package_name):
            component_registry, _ = get_resource(plugin_ressource, package_name)
            package_plugins += [line.split(';')[0] for line in component_registry.splitlines()]
    return package_plugins


def get_registered_plugin_ressources():
    """
    Get all plugin ressources registered in the ament index.

    :return: a list of (package name, plugin type names) tuples.
    """
    return [
        (package_name, get_package_plugin_ressource(package_name=package_name))
        for package_name in get_package_names_with_plugin_ressource_types()
    ]
