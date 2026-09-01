from setuptools import find_packages
from setuptools import setup

package_name = 'ros2log'

setup(
    name=package_name,
    version='0.42.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    package_data={'': ['py.typed']},
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Tomoya Fujita, Fumiya Ohnishi',
    author_email='tomoya.fujita825@gmail.com, fumiya-onishi@keio.jp',
    maintainer='Tomoya Fujita',
    maintainer_email='tomoya.fujita825@gmail.com',
    url='https://github.com/ros2/ros2cli/tree/master/ros2log',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'Programming Language :: Python',
    ],
    description='The log command for ROS 2 command line tools.',
    long_description="""\
The package provides the log command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'ros2cli.command': [
            'log = ros2log.command.log:LogCommand',
        ],
        'ros2cli.extension_point': [
            'ros2log.verb = ros2log.verb:VerbExtension',
        ],
        'ros2log.verb': [
            'get = ros2log.verb.get:GetVerb',
            'levels = ros2log.verb.levels:LevelsVerb',
            'list = ros2log.verb.list:ListVerb',
            'set = ros2log.verb.set:SetVerb',
            'watch = ros2log.verb.watch:WatchVerb',
        ],
    },
)
