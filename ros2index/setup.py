from setuptools import find_packages
from setuptools import setup

package_name = 'ros2index'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Thomas Ung',
    author_email='thomas.ung@pal-robotics.com',
    maintainer='Thomas Ung',
    maintainer_email='thomas.ung@pal-robotics.com',
    url='https://github.com/ros2/ros2cli/tree/rolling/ros2index',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Programming Language :: Python',
    ],
    description='The index command for ROS 2 command line tools.',
    long_description="""\
The package provides the index command for the ROS 2 command line tools,
for listing and inspecting entries in the ament resource index.""",
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'ros2cli.command': [
            'index = ros2index.command.index:IndexCommand',
        ],
        'ros2cli.extension_point': [
            'ros2index.verb = ros2index.verb:VerbExtension',
        ],
        'ros2index.verb': [
            'get = ros2index.verb.get:GetVerb',
            'list = ros2index.verb.list:ListVerb',
            'types = ros2index.verb.types:TypesVerb',
        ],
    }
)
