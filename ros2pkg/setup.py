from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2pkg',
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    zip_safe=False,
    author='Dirk Thomas',
    author_email='dthomas@osrfoundation.org',
    maintainer='Dirk Thomas',
    maintainer_email='dthomas@osrfoundation.org',
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
        ],
    }
)
