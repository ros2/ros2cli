from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2action',
    version='0.6.3',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Jacob Perron',
    author_email='jacob@openrobotics.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
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
            # 'send_goal = ros2action.verb.send_goal:SendGoalVerb',
            'show = ros2action.verb.show:ShowVerb',
        ],
    }
)
