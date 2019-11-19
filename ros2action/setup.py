from setuptools import find_packages
from setuptools import setup

package_name = 'ros2action'

setup(
    name=package_name,
    version='0.8.5',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Jacob Perron',
    author_email='jacob@openrobotics.org',
    maintainer='Jacob Perron',
    maintainer_email='jacob@openrobotics.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2action',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The action command for ROS 2 command line tools.',
    long_description="""\
The package provides the action command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'action = ros2action.command.action:ActionCommand',
        ],
        'ros2cli.extension_point': [
            'ros2action.verb = ros2action.verb:VerbExtension',
        ],
        'ros2action.verb': [
            # 'echo = ros2action.verb.echo:EchoVerb',
            'info = ros2action.verb.info:InfoVerb',
            'list = ros2action.verb.list:ListVerb',
            'send_goal = ros2action.verb.send_goal:SendGoalVerb',
            'show = ros2action.verb.show:ShowVerb',
        ],
    }
)
