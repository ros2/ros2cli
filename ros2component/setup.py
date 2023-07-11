from setuptools import find_packages
from setuptools import setup

package_name = 'ros2component'

setup(
    name=package_name,
    version='0.28.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Michel Hidalgo',
    author_email='michel@ekumenlabs.com',
    maintainer='Audrow Nash, Geoffrey Biggs',
    maintainer_email='audrow@openrobotics.org, geoff@openrobotics.org',
    url='https://github.com/ros2/ros2cli/tree/master/ros2component',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The component command for ROS 2 command line tools.',
    long_description="""\
The package provides the component command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'component = ros2component.command.component:ComponentCommand',
        ],
        'ros2cli.extension_point': [
            'ros2component.verb = ros2component.verb:VerbExtension',
        ],
        'ros2component.verb': [
            'list = ros2component.verb.list:ListVerb',
            'load = ros2component.verb.load:LoadVerb',
            'standalone = ros2component.verb.standalone:StandaloneVerb',
            'types = ros2component.verb.types:TypesVerb',
            'unload = ros2component.verb.unload:UnloadVerb',
        ],
    }
)
