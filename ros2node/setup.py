from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2node',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2node',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The node command for ROS 2 command line tools.',
    long_description="""\
The package provides the node command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'ros2cli.command': [
            'node = ros2node.command.node:NodeCommand',
        ],
        'ros2cli.extension_point': [
            'ros2node.verb = ros2node.verb:VerbExtension',
        ],
        'ros2node.verb': [
            'list = ros2node.verb.list:ListVerb',
        ],
    }
)
