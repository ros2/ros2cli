from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2service',
    version='0.7.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    zip_safe=True,
    author='William Woodall',
    author_email='william@osrfoundation.org',
    maintainer='William Woodall',
    maintainer_email='william@osrfoundation.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2service',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The service command for ROS 2 command line tools.',
    long_description="""\
The package provides the service command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'service = ros2service.command.service:ServiceCommand',
        ],
        'ros2cli.extension_point': [
            'ros2service.verb = ros2service.verb:VerbExtension',
        ],
        'ros2service.verb': [
            'call = ros2service.verb.call:CallVerb',
            'find = ros2service.verb.find:FindVerb',
            'list = ros2service.verb.list:ListVerb',
            'type = ros2service.verb.type:TypeVerb',
        ],
    }
)
