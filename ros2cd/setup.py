from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2cd',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    author='Vincent Rousseau',
    author_email='vincent.rousseau@irstea.fr',
    maintainer='Vincent Rousseau',
    maintainer_email='vincent.rousseau@irstea.fr',
    url='https://github.com/ros2/ros2cli/tree/master/ros2cd',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The cd command for ROS 2 command line tools.',
    long_description="""\
The package provides the cd command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'cd = ros2cd.command.cd:CdCommand',
        ],
    }
)
