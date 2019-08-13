from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2debug',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Claire Wang',
    author_email='clairewang@openrobotics.org',
    maintainer='Claire Wang',
    maintainer_email='clairewang@openrobotics.org',
    url='',
    download_url='',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='The debug command for ROS 2 command line tools',
    long_description="""\
    The package provides the debug command for ROS 2 command line tools""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'debug = ros2debug.command.debug:DebugCommand',
        ],
        'ros2cli.extension_point': [
            'ros2debug.verb = ros2debug.verb:VerbExtension',
        ],
        'ros2debug.verb': [
            'setup = ros2debug.verb.setup:SetupVerb',
        ],
    }
)
