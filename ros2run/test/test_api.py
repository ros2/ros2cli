# Copyright 2026 Open Source Robotics Foundation, Inc.
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


import sys
from types import SimpleNamespace
from unittest import mock


def _run_executable_cmd(tmp_path, *, name, content, argv, prefix=None,
                        os_name='nt'):
    # Build a file on disk, run run_executable while pretending to be on a
    # given OS, and return the command handed to subprocess.Popen.
    #
    # os.name is simulated by rebinding the module-level 'os' reference inside
    # ros2run.api to a stand-in; the shared os.name attribute is never touched.
    from ros2run import api

    script = tmp_path / name
    script.write_bytes(content)

    fake_os = SimpleNamespace(name=os_name)
    with mock.patch.object(api, 'os', fake_os), \
            mock.patch.object(api.signal, 'signal'), \
            mock.patch.object(api.subprocess, 'Popen') as popen:
        popen.return_value.returncode = 0
        rc = api.run_executable(path=str(script), argv=list(argv),
                                prefix=prefix)
    assert rc == 0
    assert popen.call_count == 1
    return popen.call_args.args[0]


PYTHON_SHEBANGS = [
    b'#!/usr/bin/env python3\n',
    b'#!/usr/bin/python3\n',
    b'#!/usr/bin/env python\n',
    b'#!/usr/bin/python\n',
    b'#!/usr/bin/env -S python3 -u\n',
    b'#!C:\\Path\\To\\python.exe\n',
    b'#!"C:\\Program Files\\Python311\\python.exe"\n',
    b'#!"C:\\Program Files\\Python311\\python.exe" -u\n',
    b'#!/a/very/long/path/that/keeps/going/and/going/and/going/to/python3\n',
]

NON_PYTHON_SHEBANGS = [
    b'#!/bin/bash\n',
    # only an argument mentions python, the interpreter is /bin/sh
    b'#!/bin/sh -c "echo python"\n',
    b'#!/usr/bin/env ruby\n',
    # 'env' options that consume an argument: '-u NAME' and '-C DIR' must not
    # let that argument (PYTHON/python) be mistaken for the interpreter (ruby)
    b'#!/usr/bin/env -S -u PYTHON ruby\n',
    b'#!/usr/bin/env -S -C python ruby\n',
    b'MZ\x90\x00\x03\x00\x00\x00',  # native Windows executable header
    b'\x7fELF\x02\x01\x01\x00binary',  # ELF binary
    b'plain text, no shebang\n',
    b'',
]


def test_shebang_invokes_python():
    from ros2run.api import _shebang_invokes_python
    # the stored lines include the leading '#!' and trailing newline
    for line in PYTHON_SHEBANGS:
        shebang = line[2:].decode().strip()
        assert _shebang_invokes_python(shebang), line
    for line in NON_PYTHON_SHEBANGS:
        if not line.startswith(b'#!'):
            continue
        shebang = line[2:].split(b'\n', 1)[0].decode('utf-8', 'replace')
        assert not _shebang_invokes_python(shebang), line


def test_has_python_shebang(tmp_path):
    from ros2run.api import _has_python_shebang

    for i, line in enumerate(PYTHON_SHEBANGS):
        p = tmp_path / f'py_{i}'
        p.write_bytes(line)
        assert _has_python_shebang(str(p)), line

    for i, line in enumerate(NON_PYTHON_SHEBANGS):
        p = tmp_path / f'nonpy_{i}'
        p.write_bytes(line)
        assert not _has_python_shebang(str(p)), line

    # non-UTF-8 bytes on the shebang line must not raise
    latin1 = tmp_path / 'latin1'
    latin1.write_bytes(b'#!/usr/bin/\xe9/python3\n')
    assert _has_python_shebang(str(latin1))

    # missing / unreadable file returns False instead of raising
    assert not _has_python_shebang(str(tmp_path / 'does_not_exist'))


def test_env_option_arguments_not_mistaken_for_python():
    from ros2run.api import _shebang_invokes_python
    # the interpreter is ruby; 'python'/'PYTHON' are arguments to env options
    assert not _shebang_invokes_python('/usr/bin/env -S -u PYTHON ruby')
    assert not _shebang_invokes_python('/usr/bin/env -S -C python ruby')
    # a genuine Python interpreter after a consuming option is still detected
    assert _shebang_invokes_python('/usr/bin/env -u SOMEVAR python3')


def test_run_executable_windows_extensionless_python_script(tmp_path):
    cmd = _run_executable_cmd(
        tmp_path, name='rqt_bag', content=b'#!/usr/bin/python3\nprint(1)\n',
        argv=['--arg'])
    assert cmd == [sys.executable, str(tmp_path / 'rqt_bag'), '--arg']


def test_run_executable_windows_dot_py_unchanged(tmp_path):
    cmd = _run_executable_cmd(
        tmp_path, name='script.py', content=b'print(1)\n', argv=['--arg'])
    assert cmd == [sys.executable, str(tmp_path / 'script.py'), '--arg']


def test_run_executable_windows_native_executable(tmp_path):
    cmd = _run_executable_cmd(
        tmp_path, name='tool.exe', content=b'MZ\x90\x00binary', argv=['--arg'])
    assert cmd == [str(tmp_path / 'tool.exe'), '--arg']


def test_run_executable_windows_non_python_script(tmp_path):
    cmd = _run_executable_cmd(
        tmp_path, name='wrapper', content=b'#!/bin/bash\necho python\n',
        argv=['--arg'])
    assert cmd == [str(tmp_path / 'wrapper'), '--arg']


def test_run_executable_windows_prefix_and_argv_order(tmp_path):
    cmd = _run_executable_cmd(
        tmp_path, name='rqt_bag', content=b'#!/usr/bin/python3\n',
        argv=['a', 'b'], prefix=['gdb', '--args'])
    assert cmd == [
        'gdb', '--args', sys.executable, str(tmp_path / 'rqt_bag'), 'a', 'b']


def test_run_executable_linux_unchanged(tmp_path):
    # On POSIX the shebang is honored by the OS; the interpreter must not be
    # prepended even for an extensionless Python script.
    cmd = _run_executable_cmd(
        tmp_path, name='rqt_bag', content=b'#!/usr/bin/python3\n',
        argv=['a', 'b'], prefix=['gdb', '--args'], os_name='posix')
    assert cmd == ['gdb', '--args', str(tmp_path / 'rqt_bag'), 'a', 'b']


def test_executable_name_completer():
    from unittest.mock import Mock, patch
    from ros2pkg.api import PackageNotFound
    from ros2run.api import ExecutableNameCompleter

    parsed_args = Mock()
    parsed_args.package_name = 'my_package'
    completer = ExecutableNameCompleter(package_name_key='package_name')

    # Test 1: package has executables
    with patch('ros2run.api.get_executable_paths') as mock_get_paths:
        mock_get_paths.return_value = [
            '/opt/ros/humble/lib/my_package/my_exec',
            '/opt/ros/humble/lib/my_package/other_exec',
        ]
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == ['my_exec', 'other_exec']
        mock_get_paths.assert_called_once_with(package_name='my_package')

    # Test 2: package not found raises PackageNotFound, returns empty list
    with patch('ros2run.api.get_executable_paths') as mock_get_paths:
        mock_get_paths.side_effect = PackageNotFound('my_package')
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == []

    # Test 3: package has no executables
    with patch('ros2run.api.get_executable_paths') as mock_get_paths:
        mock_get_paths.return_value = []
        result = completer(prefix='', parsed_args=parsed_args)
        assert result == []
