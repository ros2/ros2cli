from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2cli',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2cli',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='Framework for ROS 2 command line tools.',
    long_description="""\
The framework provides a single command line script which can be extended with
commands and verbs.""",
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'ros2cli.command': [
        ],
        'ros2cli.extension_point': [
            'ros2cli.command = ros2cli.command:CommandExtension',
        ],
        'console_scripts': [
            'ros2 = ros2cli.cli:main',
        ],
    }
)
