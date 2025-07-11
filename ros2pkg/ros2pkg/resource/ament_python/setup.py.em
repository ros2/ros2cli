from setuptools import find_packages, setup

package_name = '@project_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='@maintainer_name',
    maintainer_email='@maintainer_email',
    description='@package_description',
    license='@package_license',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
@[if node_name]@
            '@node_name = @project_name.@node_name:main'
@[end if]@
        ],
    },
)
