# Copyright 2016-2017 Dirk Thomas
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

from collections import OrderedDict
import logging

from pkg_resources import parse_version

from ros2cli.entry_points import load_entry_points

PLUGIN_SYSTEM_VERSION = '0.1'

logger = logging.getLogger(__name__)


class PluginException(Exception):
    """Base class for all exceptions within the plugin system."""

    pass


_extension_instances = {}


def instantiate_extensions(
    group_name, *, exclude_names=None, unique_instance=False
):
    extension_types = load_entry_points(group_name)
    extension_instances = {}
    for extension_name, extension_class in extension_types.items():
        if exclude_names and extension_name in exclude_names:
            continue
        extension_instance = _instantiate_extension(
            group_name, extension_name, extension_class,
            unique_instance=unique_instance)
        if extension_instance is None:
            continue
        extension_instances[extension_name] = extension_instance
    return extension_instances


def _instantiate_extension(
    group_name, extension_name, extension_class, *, unique_instance=False
):
    global _extension_instances
    if not unique_instance and extension_class in _extension_instances:
        return _extension_instances[extension_class]

    try:
        extension_instance = extension_class()
    except PluginException as e:
        logger.warn(
            "Failed to instantiate '{group_name}' extension "
            "'{extension_name}': {e}".format_map(locals()))
        return None
    except Exception as e:
        logger.error(
            "Failed to instantiate '{group_name}' extension "
            "'{extension_name}': {e}".format_map(locals()))
        return None
    if not unique_instance:
        _extension_instances[extension_class] = extension_instance
    return extension_instance


def order_extensions(extensions, key_function, *, reverse=False):
    return OrderedDict(
        sorted(extensions.items(), key=key_function, reverse=reverse))


def order_extensions_by_name(extensions):
    return order_extensions(extensions, lambda pair: pair[0])


def satisfies_version(version, caret_range):
    assert caret_range.startswith('^'), 'Only supports caret ranges'
    extension_point_version = parse_version(version)
    extension_version = parse_version(caret_range[1:])
    next_extension_version = get_upper_bound_caret_version(
        extension_version)

    if extension_point_version < extension_version:
        raise PluginException(
            "Extension point is too old (%s), the extension requires "
            "'%s'" % (extension_point_version, extension_version))

    if extension_point_version >= next_extension_version:
        raise PluginException(
            "Extension point is newer (%s), than what the extension "
            "supports '%s'" % (extension_point_version, extension_version))


def get_upper_bound_caret_version(version):
    parts = version.base_version.split('.')
    if len(parts) < 2:
        parts += [0] * (2 - len(parts))
    major, minor = [int(p) for p in parts[:2]]
    if major > 0:
        major += 1
        minor = 0
    else:
        minor += 1
    return parse_version('%d.%d.0' % (major, minor))
