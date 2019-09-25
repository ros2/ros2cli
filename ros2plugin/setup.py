from setuptools import find_packages
from setuptools import setup

package_name = 'ros2plugin'

setup(
    name=package_name,
    version='0.7.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Jeremie Deray',
    author_email='jeremie.deray@canonical.com',
    maintainer='Jeremie Deray',
    maintainer_email='jeremie.deray@canonical.com',
    url='https://github.com/ros2/ros2cli/tree/master/ros2plugin',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The plugin command for ROS 2 command line tools.',
    long_description="""\
The package provides the plugin command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'plugin = ros2plugin.command.plugin:PluginCommand',
        ],
        'ros2cli.extension_point': [
            'ros2plugin.verb = ros2plugin.verb:VerbExtension',
        ],
        'ros2plugin.verb': [
            'list = ros2plugin.verb.list:ListVerb',
        ],
    }
)
