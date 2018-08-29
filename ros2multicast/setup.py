from setuptools import setup

setup()

# from setuptools import find_packages
# from setuptools import setup

# setup(
#     name='ros2multicast',
#     version='0.5.4',
#     packages=find_packages(exclude=['test']),
#     install_requires=['ros2cli'],
#     zip_safe=True,
#     author='Dirk Thomas',
#     author_email='dthomas@osrfoundation.org',
#     maintainer='Dirk Thomas',
#     maintainer_email='dthomas@osrfoundation.org',
#     url='https://github.com/ros2/ros2cli/tree/master/ros2multicast',
#     download_url='https://github.com/ros2/ros2cli/releases',
#     keywords=[],
#     classifiers=[
#         'Environment :: Console',
#         'Intended Audience :: Developers',
#         'License :: OSI Approved :: Apache Software License',
#         'Programming Language :: Python',
#     ],
#     description='The multicast command for ROS 2 command line tools.',
#     long_description="""\
# The package provides the multicast command for the ROS 2 command line tools.""",
#     license='Apache License, Version 2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'ros2cli.command': [
#             'multicast = ros2multicast.command.multicast:MulticastCommand',
#         ],
#         'ros2cli.extension_point': [
#             'ros2multicast.verb = ros2multicast.verb:VerbExtension',
#         ],
#         'ros2multicast.verb': [
#             'receive = ros2multicast.verb.receive:ReceiveVerb',
#             'send = ros2multicast.verb.send:SendVerb',
#         ],
#     }
# )
