from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2topic',
    version='0.5.2',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2topic',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The topic command for ROS 2 command line tools.',
    long_description="""\
The package provides the topic command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'topic = ros2topic.command.topic:TopicCommand',
        ],
        'ros2cli.extension_point': [
            'ros2topic.verb = ros2topic.verb:VerbExtension',
        ],
        'ros2topic.verb': [
            'echo = ros2topic.verb.echo:EchoVerb',
            'info = ros2topic.verb.info:InfoVerb',
            'list = ros2topic.verb.list:ListVerb',
            'pub = ros2topic.verb.pub:PubVerb',
        ],
    }
)
