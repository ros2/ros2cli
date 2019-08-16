from setuptools import find_packages
from setuptools import setup

setup(
    name='ros2doctor',
    version='0.7.4',
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
    description='The doctor command for ROS 2 command line tools',
    long_description="""\
    The package provides a cli tool to check potential issues in ROS2 system""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'doctor = ros2doctor.command.doctor:DoctorCommand',
            'wtf = ros2doctor.command.doctor:WtfCommand',
        ],
        'ros2doctor.api':[
            'check_platform = ros2doctor.api.platform:check_platform',
        ],
    }
)
