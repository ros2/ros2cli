from setuptools import find_packages
from setuptools import setup

package_name = 'ros2param'

setup(
    name=package_name,
    version='0.17.1',
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
    maintainer='Aditya Pande, Audrow Nash, Michael Jeronimo',
    maintainer_email='aditya.pande@openrobotics.org, audrow@openrobotics.org, michael.jeronimo@openrobotics.org',  # noqa: E501
    url='https://github.com/ros2/ros2cli/tree/master/ros2param',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The param command for ROS 2 command line tools.',
    long_description="""\
The package provides the param command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'param = ros2param.command.param:ParamCommand',
        ],
        'ros2cli.extension_point': [
            'ros2param.verb = ros2param.verb:VerbExtension',
        ],
        'ros2param.verb': [
            'delete = ros2param.verb.delete:DeleteVerb',
            'describe = ros2param.verb.describe:DescribeVerb',
            'dump = ros2param.verb.dump:DumpVerb',
            'get = ros2param.verb.get:GetVerb',
            'list = ros2param.verb.list:ListVerb',
            'set = ros2param.verb.set:SetVerb',
            'load = ros2param.verb.load:LoadVerb',
        ],
    }
)
