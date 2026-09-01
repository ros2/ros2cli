from pathlib import Path


def _ros2cli_package_root() -> Path:
    return Path(__file__).resolve().parents[1]


def test_package_dsv_sources_powershell_completion():
    package_dsv = _ros2cli_package_root() / 'resource' / 'package.dsv'

    lines = package_dsv.read_text(encoding='utf-8').splitlines()

    assert 'source;share/ros2cli/environment/ros2-argcomplete.ps1' in lines


def test_powershell_completion_uses_full_command_line():
    completion_script = _ros2cli_package_root() / 'completion' / 'ros2-argcomplete.ps1'

    content = completion_script.read_text(encoding='utf-8')

    assert 'Register-ArgumentCompleter -Native -CommandName ros2 -ScriptBlock {' in content
    assert 'param($wordToComplete, $commandLine, $cursorPosition)' in content
    assert '$env:COMP_LINE = $commandLine' in content
