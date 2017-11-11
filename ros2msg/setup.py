from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2msg',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2msg',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The msg command for ROS 2 command line tools.',
    long_description="""\
The package provides the msg command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'msg = ros2msg.command.msg:MsgCommand',
        ],
        'ros2cli.extension_point': [
            'ros2msg.verb = ros2msg.verb:VerbExtension',
        ],
        'ros2msg.verb': [
            'list = ros2msg.verb.list:ListVerb',
            'package = ros2msg.verb.package:PackageVerb',
            'packages = ros2msg.verb.packages:PackagesVerb',
            'show = ros2msg.verb.show:ShowVerb',
        ],
    }
)
