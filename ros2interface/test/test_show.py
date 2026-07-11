# Copyright 2026 Old-Ding
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

"""Tests for the interface show verb."""

from ros2interface.verb import show


def test_show_msg_interface(tmp_path, monkeypatch, capsys):
    msg_text = 'uint32 value\n'
    msg_path = tmp_path / 'Basic.msg'
    msg_path.write_text(msg_text)
    monkeypatch.setattr(show, 'get_interface_path', lambda _: str(msg_path))

    show._show_interface('test_interfaces/msg/Basic')

    assert capsys.readouterr().out == msg_text


def test_show_idl_interface(tmp_path, monkeypatch, capsys):
    idl_text = """\
module test_interfaces {
  module msg {
    struct IdlOnly {
      uint32 value;
    };
  };
};
"""
    idl_path = tmp_path / 'IdlOnly.idl'
    monkeypatch.setattr(show, 'get_interface_path', lambda _: str(idl_path))

    for file_text in (idl_text, idl_text.rstrip('\n')):
        idl_path.write_text(file_text)
        show._show_interface('test_interfaces/msg/IdlOnly')

        assert capsys.readouterr().out == idl_text
