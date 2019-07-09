from setuptools import setup

package_name = '@project_name'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='@maintainer_name',
    maintainer_email='@maintainer_email',
    license='@package_license',
    entry_points={
        'console_scripts': [
            '@node_name = @project_name.@node_name:main'
        ],
    },
)