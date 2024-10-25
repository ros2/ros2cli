# ros2doctor

This folder contains the source code for ros2doctor.
It is one of the ROS 2 command line interface tools included with a standard install of any ROS 2 distro.
`ros2doctor` is similar to `roswtf` from ROS 1.
It will examine your ROS 2 setup, such as distribution, platform, network interface, etc., and look for potential issues in a running ROS 2 system.

## Usage

Run `ros2 doctor` or `ros2 wtf`(alias) to conduct checks.

Run `ros2 doctor -h/--help` to print all available command arguments.

Run `ros2 doctor -r/--report` to see report of all checked items.

Run `ros2 doctor -rf/--report-fail` to see report of failed checks only.

Run `ros2 doctor -iw/--include-warnings` to include warnings as failed checks.
`-iw` and `-rf` can be used in combination.

Run `ros2 doctor -ep/--exclude-packages` to exclude package checks or report.


## Add New Checks

To add your own checks or information to report, use [Python entry points](https://setuptools.readthedocs.io/en/latest/pkg_resources.html#entry-points) to add modules to `setup.py`.
See example below:

```python
    entry_points={
        'ros2doctor.checks': [
            'PlatformCheck = ros2doctor.api.platform:PlatformCheck',
            'NetworkCheck = ros2doctor.api.network:NetworkCheck',
        ],
        'ros2doctor.report': [
            'PlatformReport = ros2doctor.api.platform:PlatformReport',
            'NetworkReport = ros2doctor.api.network:NetworkReport',
        ],
    }
```
