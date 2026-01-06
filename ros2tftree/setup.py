from setuptools import find_packages
from setuptools import setup

package_name = 'ros2tftree'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    package_data={'': ['py.typed']},
    install_requires=['ros2cli', 'tf_tree_terminal'],
    zip_safe=True,
    description='The tf-tree command for ROS 2 command line tools.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'ros2cli.command': [
            'tf-tree = ros2tftree.command.tf_tree:TfTreeCommand',
        ],
        'ros2cli.extension_point': [
            'ros2tftree.verb = ros2tftree.verb:VerbExtension',
        ],
        'ros2tftree.verb': [
            'show = ros2tftree.verb.show:ShowVerb',
        ],
    }
)
