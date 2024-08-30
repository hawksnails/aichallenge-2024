from setuptools import setup, find_packages
import os
import glob
package_name = 'path_to_trajectory'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='name',
    maintainer_email='name@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_to_trajectory_node = path_to_trajectory.path_to_trajectory_node:main',
            'set_route_client = path_to_trajectory.set_route_client:main',
        ],
    },
)
