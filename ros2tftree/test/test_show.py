import sys
import types

from argparse import ArgumentParser

from ros2tftree.command.tf_tree import TfTreeCommand


def test_tf_tree_command_forwards_args(monkeypatch):
    # monkeypatch tf_tree_terminal.main to capture argv
    called = {}

    def fake_main():
        called['argv'] = sys.argv[1:]

    mod = types.ModuleType('tf_tree_terminal.tree_node')
    mod.main = fake_main
    monkeypatch.setitem(sys.modules, 'tf_tree_terminal', types.ModuleType('tf_tree_terminal'))
    monkeypatch.setitem(sys.modules, 'tf_tree_terminal.tree_node', mod)

    parser = ArgumentParser()
    cmd = TfTreeCommand()
    # pass an empty argv to avoid pytest's args leaking into parsing
    cmd.add_arguments(parser, 'ros2 tf-tree', argv=[])

    args = parser.parse_args(['-a', '-l', '-c', '-nc'])
    # emulate command behavior when no subcommand is present
    result = cmd.main(parser=parser, args=args)
    assert ' -a' not in called.get('argv', [])
    # ensure the delegated args include expected flags
    for flag in ['-a', '-l', '-c', '--no-color']:
        assert flag in called['argv']
