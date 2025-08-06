from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2cli_test_interfaces',
    version='0.40.0',
    packages=find_packages(
        include=('ros2cli_test_interfaces', 'ros2cli_test_interfaces.*')),
)
