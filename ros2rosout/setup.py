from setuptools import find_packages
from setuptools import setup

package_name = 'ros2rosout'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['ros2cli'],
    zip_safe=True,
    author='Guillaume Autran',
    author_email='gautran@clearpath.ai',
    maintainer='Guillaume Autran',
    maintainer_email='gautran@clearpath.ai',
    url='',
    download_url='',
    keywords=[],
    classifiers=[
        'Environment :: Console',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
    ],
    description='A convenient command to display the /rosout logs for ROS 2 command line tools',
    long_description="""\
    The package provides a cli tool to print the `/rosout` logs in a ROS 2 system""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'rosout = ros2rosout.command.rosout:RosoutCommand',
        ],
        'ros2cli.extension_point': [
            'ros2rosout.verb = ros2rosout.verb:VerbExtension',
        ],
        'ros2rosout.verb': [
            'print = ros2rosout.verb.print:PrintVerb'
        ]
    }
)
