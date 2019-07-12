# ros2cli

This repository contains the source code for ROS 2 command line interface tools included with a standard install of any ROS 2 distro.

## Usage

Run `ros2 --help` to see all available commands.

Run `ros2 <command> --help` for more information on individual command usage.

Run `ros2 <command> <verb> --help` for even more usage information on a specific command's verbs.   

Read [Introspection with command line tools](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools/) on ROS Index for more information and an example.

### Cheat Sheet

This [cheat sheet](https://github.com/artivis/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf) provides examples for commands and their verbs (some of which rely on the [ROS 2 demos package](https://github.com/ros2/demos)).

## Add New Verbs

You can use [Python entry points](https://setuptools.readthedocs.io/en/latest/pkg_resources.html#entry-points) to create new commands and verbs for the CLI.
[Here's an example](https://github.com/ros2/ros2cli/pull/273/files).
And [here's and example](https://github.com/artivis/ros2hellocli) for how to add a command.

## Background

You can find a [historical discussion](https://discourse.ros.org/t/ros-graph-information-tools-implementation-discussion/674/34) on this subject on Discourse.
