# Copyright 2026 Miko Parkkinen.
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

from argparse import Namespace

from ros2pkg.verb.depends import DependsVerb


PACKAGE_XML = '''\
<package format="3">
  <name>test_package</name>
  <version>0.0.0</version>
  <description>Test package</description>
  <maintainer email="test@example.com">Test</maintainer>
  <license>Apache-2.0</license>
  <depend>common_dependency</depend>
  <buildtool_depend>buildtool_dependency</buildtool_depend>
  <test_depend>test_dependency</test_depend>
  <exec_depend condition="$DEPENDENCY_ENABLED == 1">enabled_dependency</exec_depend>
  <exec_depend condition="$DEPENDENCY_ENABLED == 0">disabled_dependency</exec_depend>
</package>
'''


def test_depends(tmp_path, monkeypatch, capsys):
    (tmp_path / 'package.xml').write_text(PACKAGE_XML)
    monkeypatch.setattr(
        'ros2pkg.verb.depends.get_package_share_directory',
        lambda package_name: str(tmp_path))
    monkeypatch.setenv('DEPENDENCY_ENABLED', '1')

    result = DependsVerb().main(args=Namespace(package_name='test_package'))

    assert result is None
    assert capsys.readouterr().out.splitlines() == [
        'buildtool_dependency',
        'common_dependency',
        'enabled_dependency',
        'test_dependency',
    ]
