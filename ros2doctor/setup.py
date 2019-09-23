from setuptools import find_packages
from setuptools import setup

package_name = 'ros2doctor'

setup(
    name=package_name,
    version='0.7.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
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
    The package provides a cli tool to check potential issues in a ROS 2 system""",
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'doctor = ros2doctor.command.doctor:DoctorCommand',
            'wtf = ros2doctor.command.doctor:WtfCommand',
        ],
        'ros2doctor.checks': [
            'PlatformCheck = ros2doctor.api.platform:PlatformCheck',
            'NetworkCheck = ros2doctor.api.network:NetworkCheck',
        ],
        'ros2doctor.report': [
            'PlatformReport = ros2doctor.api.platform:PlatformReport',
            'RosdistroReport = ros2doctor.api.platform:RosdistroReport',
            'NetworkReport = ros2doctor.api.network:NetworkReport',
            'RMWReport = ros2doctor.api.rmw:RMWReport',
        ],
    }
)
