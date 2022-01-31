from setuptools import find_packages
from setuptools import setup

package_name = 'ros2lifecycle'

setup(
    name=package_name,
    version='0.9.11',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2lifecycle',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The lifecycle command for ROS 2 command line tools.',
    long_description="""\
The package provides the lifecycle command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'lifecycle = ros2lifecycle.command.lifecycle:LifecycleCommand',
        ],
        'ros2cli.extension_point': [
            'ros2lifecycle.verb = ros2lifecycle.verb:VerbExtension',
        ],
        'ros2lifecycle.verb': [
            'get = ros2lifecycle.verb.get:GetVerb',
            'list = ros2lifecycle.verb.list:ListVerb',
            'nodes = ros2lifecycle.verb.nodes:NodesVerb',
            'set = ros2lifecycle.verb.set:SetVerb',
        ],
    }
)
