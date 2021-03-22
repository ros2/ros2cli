from setuptools import setup
from ament_package.generate_setuptools_dict import generate_setuptools_dict

package_name = '@project_name'
package_info = generate_setuptools_dict(
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
@[if node_name]@
            '@node_name = @project_name.@node_name:main'
@[end if]@
        ],
    },
)
setup(**package_info)
