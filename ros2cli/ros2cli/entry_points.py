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

from collections import defaultdict
import logging

from pkg_resources import iter_entry_points
from pkg_resources import WorkingSet

"""
The group name for entry points identifying extension points.

While all entry points in this package start with ``ros2cli.`` other
distributions might define entry points with a different prefix.
Those need to be declared using this group name.
"""
EXTENSION_POINT_GROUP_NAME = 'ros2cli.extension_point'

logger = logging.getLogger(__name__)


def get_all_entry_points():
    """
    Get all entry points related to ``ros2cli`` and any of its extensions.

    :returns: mapping of entry point names to ``EntryPoint`` instances
    :rtype: dict
    """
    extension_points = get_entry_points(EXTENSION_POINT_GROUP_NAME)

    entry_points = defaultdict(dict)
    working_set = WorkingSet()
    for dist in sorted(working_set):
        entry_map = dist.get_entry_map()
        for group_name in entry_map.keys():
            # skip groups which are not registered as extension points
            if group_name not in extension_points:
                continue

            group = entry_map[group_name]
            for entry_point_name, entry_point in group.items():
                entry_points[group_name][entry_point_name] = \
                    (dist, entry_point)
    return entry_points


def get_entry_points(group_name):
    """
    Get the entry points for a specific group.

    :param str group_name: the name of the ``entry_point`` group
    :returns: mapping of group name to dictionaries which map entry point names
      to ``EntryPoint`` instances
    :rtype: dict
    """
    entry_points = {}
    for entry_point in iter_entry_points(group=group_name):
        entry_points[entry_point.name] = entry_point
    return entry_points


def load_entry_points(group_name):
    """
    Load the entry points for a specific group.

    :param str group_name: the name of the ``entry_point`` group
    :returns: mapping of entry point name to loaded entry point
    :rtype: dict
    """
    extension_types = {}
    for entry_point in get_entry_points(group_name).values():
        try:
            extension_type = entry_point.load()
        except Exception as e:  # noqa: F841
            logger.warn(
                "Failed to load entry point '{entry_point.name}': {e}"
                .format_map(locals()))
            continue
        extension_types[entry_point.name] = extension_type
    return extension_types


def get_first_line_doc(any_type):
    if not any_type.__doc__:
        return ''
    lines = any_type.__doc__.splitlines()
    if not lines:
        return ''
    if lines[0]:
        line = lines[0]
    elif len(lines) > 1:
        line = lines[1]
    return line.strip().rstrip('.')
