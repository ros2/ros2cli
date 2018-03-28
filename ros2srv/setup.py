from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2srv',
    version='0.4.1',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2srv',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The srv command for ROS 2 command line tools.',
    long_description="""\
The package provides the srv command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'srv = ros2srv.command.srv:SrvCommand',
        ],
        'ros2cli.extension_point': [
            'ros2srv.verb = ros2srv.verb:VerbExtension',
        ],
        'ros2srv.verb': [
            'list = ros2srv.verb.list:ListVerb',
            'package = ros2srv.verb.package:PackageVerb',
            'packages = ros2srv.verb.packages:PackagesVerb',
            'show = ros2srv.verb.show:ShowVerb',
        ],
    }
)
