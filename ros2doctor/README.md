# ros2doctor

This folder contains the source code of ros2doctor, part of the ROS 2 command line interface tools included with a standard install of any ROS 2 distro. ros2doctor command line tool is similar to roswtf in ROS system. It will examine your ROS 2 setup, such as distribution and platform, environment variables, network etc. and look for potential issues.

## Usage

Run `ros2 doctor` or `ros2 wtf`(alias) to conduct checks.

Run `ros2 doctor -h/--help` to print all available command extensions.

Run `ros2 doctor -r/--report` to see details of checks.

## Add New Checks

To add your own checks or information to report, use [Python entry points](https://setuptools.readthedocs.io/en/latest/pkg_resources.html#entry-points) to add modules to `setup.py`. See example below:

```python
    entry_points={
        'ros2doctor.checks': [
            'check_platform = ros2doctor.api.platform:check_platform',
        ],
        'ros2doctor.report': [
            'report_platform = ros2doctor.api.platform:print_platform_info',
            'report_ros_distro = ros2doctor.api.platform:print_ros2_info',
        ],
    }
```
