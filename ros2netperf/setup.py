from setuptools import find_packages
from setuptools import setup

package_name = 'ros2netperf'

setup(
    name=package_name,
    version='0.19.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Ivan Santiago Paunovic',
    author_email='ivanpauno@ekumenlabs.com',
    maintainer='Aditya Pande, Audrow Nash, Michael Jeronimo',
    maintainer_email='aditya.pande@openrobotics.org, audrow@openrobotics.org, michael.jeronimo@openrobotics.org',  # noqa: E501
    url='https://github.com/ros2/ros2cli/tree/master/ros2topic',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The perf command for ROS 2 command line tools.',
    long_description="""\
The package provides the perf command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'perf = ros2netperf.command.perf:PerfCommand',
        ],
        'ros2cli.extension_point': [
            'ros2netperf.verb = ros2netperf.verb:VerbExtension',
        ],
        'ros2netperf.verb': [
            'client = ros2netperf.verb.client:ClientVerb',
            'server = ros2netperf.verb.server:ServerVerb',
        ],
    }
)
