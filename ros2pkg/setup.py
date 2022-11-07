from setuptools import find_packages
from setuptools import setup

package_name = 'ros2pkg'

setup(
    name=package_name,
    version='0.18.4',
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
    url='https://github.com/ros2/ros2cli/tree/master/ros2pkg',
    download_url='https://github.com/ros2/ros2cli/releases',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The pkg command for ROS 2 command line tools.',
    long_description="""\
The package provides the pkg command for the ROS 2 command line tools.""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'pkg = ros2pkg.command.pkg:PkgCommand',
        ],
        'ros2cli.extension_point': [
            'ros2pkg.verb = ros2pkg.verb:VerbExtension',
        ],
        'ros2pkg.verb': [
            'create = ros2pkg.verb.create:CreateVerb',
            'executables = ros2pkg.verb.executables:ExecutablesVerb',
            'list = ros2pkg.verb.list:ListVerb',
            'prefix = ros2pkg.verb.prefix:PrefixVerb',
            'xml = ros2pkg.verb.xml:XmlVerb',
        ],
    },
    package_data={
        'ros2pkg': [
            'resource/**/*',
        ],
    },
)
