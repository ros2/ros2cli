from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2cli',
    version='0.35.0',
    packages=find_packages(exclude=['test']),
    extras_require={
        'completion': ['argcomplete'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/ros2cli',
        ]),
        ('share/ros2cli', [
            'package.xml',
            'resource/package.dsv',
        ]),
        ('share/ros2cli/environment', [
            'completion/ros2-argcomplete.bash',
            'completion/ros2-argcomplete.zsh'
        ]),
    ],
    zip_safe=False,
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Audrow Nash, Geoffrey Biggs',
    maintainer_email='audrow@openrobotics.org, geoff@openrobotics.org',
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
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'daemon = ros2cli.command.daemon:DaemonCommand',
            'extension_points ='
            ' ros2cli.command.extension_points:ExtensionPointsCommand',
            'extensions = ros2cli.command.extensions:ExtensionsCommand',
        ],
        'ros2cli.extension_point': [
            'ros2cli.command = ros2cli.command:CommandExtension',
            'ros2cli.daemon.verb = ros2cli.verb.daemon:VerbExtension',
        ],
        'ros2cli.daemon.verb': [
            'start = ros2cli.verb.daemon.start:StartVerb',
            'status = ros2cli.verb.daemon.status:StatusVerb',
            'stop = ros2cli.verb.daemon.stop:StopVerb',
        ],
        'console_scripts': [
            'ros2 = ros2cli.cli:main',
            '_ros2_daemon = ros2cli.daemon:main',
        ],
    }
)
