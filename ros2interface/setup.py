from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2interface',
    version='0.7.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Siddharth Kucheria',
    author_email='kucheria@osrfoundation.org',
    maintainer='Siddharth Kucheria',
    maintainer_email='kucheria@osrfoundation.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2msg',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The interface command for ROS 2 command line tools.',
    long_description="""\
The package provides the interface command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'interface = ros2interface.command.interface:InterfaceCommand',
        ],
        'ros2cli.extension_point': [
            'ros2interface.verb = ros2interface.verb:VerbExtension',
        ],
        'ros2interface.verb': [
            'list = ros2interface.verb.list:ListVerb',
            'packages = ros2interface.verb.packages:PackagesVerb',
            'package = ros2interface.verb.package:PackageVerb',
            'show = ros2interface.verb.show:ShowVerb',
            'info = ros2interface.verb.info:InfoVerb',
        ],
    }
)
