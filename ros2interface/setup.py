from setuptools import find_packages
from setuptools import setup

package_name = 'ros2interface'

setup(
    name=package_name,
    version='0.16.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Siddharth Kucheria, Jacob Perron',
    author_email='kucheria@osrfoundation.org, jacob@osrfoundation.org',
    maintainer='Aditya Pande, Audrow Nash, Michael Jeronimo',
    maintainer_email='aditya.pande@openrobotics.org, audrow@openrobotics.org, michael.jeronimo@openrobotics.org',  # noqa: E501
    url='https://github.com/ros2/ros2cli/tree/master/ros2interface',
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
            'package = ros2interface.verb.package:PackageVerb',
            'proto = ros2interface.verb.proto:ProtoVerb',
            'packages = ros2interface.verb.packages:PackagesVerb',
            'show = ros2interface.verb.show:ShowVerb',
        ],
    }
)
