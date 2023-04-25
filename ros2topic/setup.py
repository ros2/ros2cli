from setuptools import find_packages
from setuptools import setup

package_name = 'ros2topic'

setup(
    name=package_name,
    version='0.18.6',
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
            'bw = ros2topic.verb.bw:BwVerb',
            'delay = ros2topic.verb.delay:DelayVerb',
            'echo = ros2topic.verb.echo:EchoVerb',
            'find = ros2topic.verb.find:FindVerb',
            'hz = ros2topic.verb.hz:HzVerb',
            'info = ros2topic.verb.info:InfoVerb',
            'list = ros2topic.verb.list:ListVerb',
            'pub = ros2topic.verb.pub:PubVerb',
            'type = ros2topic.verb.type:TypeVerb',
        ],
    }
)
