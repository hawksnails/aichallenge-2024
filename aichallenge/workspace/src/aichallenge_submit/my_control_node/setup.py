from setuptools import setup, find_packages

package_name = 'my_control_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mtsubaki',
    maintainer_email='tsubaki@g.ecc.u-tokyo.ac.jp',
    description='A ROS 2 node that subscribes to control commands and modifies them.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_node = my_control_node.control_node:main',
        ],
    },
    package_data={package_name: ['package.xml']},
    include_package_data=True,
)
