# ros2cli

This repository contains the source code for ROS 2 command line interface tools included with a standard install of any ROS 2 distro. 

## Usage

Run `ros2 --help` to see all available verbs.

Run `ros2 <verb> --help` for more information on individual command usage.

Read [Introspection with command line tools](https://index.ros.org/doc/ros2/Tutorials/Introspection-with-command-line-tools/) on ROS Index for more information and an example.

### Cheat Sheet

This [cheat sheet](https://github.com/artivis/ros2_cheats_sheet/blob/master/cli/cli_cheats_sheet.pdf) provides examples for verbs and their sub-verbs (some of which rely on the [ROS 2 demos package](https://github.com/ros2/demos)).

## Add New Verbs

You can use [Python entry points](https://setuptools.readthedocs.io/en/latest/pkg_resources.html#entry-points) to create new verbs and sub-verbs for the cli. 
[Here's an example](https://github.com/ros2/ros2cli/pull/273/files).

## Background

You can find a [historical discussion](https://discourse.ros.org/t/ros-graph-information-tools-implementation-discussion/674/34) on this subject on Discourse.
