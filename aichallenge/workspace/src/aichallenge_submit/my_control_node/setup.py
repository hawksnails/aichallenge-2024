from setuptools import setup, find_packages

package_name = 'my_control_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    install_requires=[
        'setuptools',
        'rclpy',
        'tier4_control_msgs',
        'autoware_auto_control_msgs',
        'autoware_auto_planning_msgs',
        'motion_utils',
    ],
    zip_safe=True,
    maintainer='mtsubaki',
    maintainer_email='tsubaki@g.ecc.u-tokyo.ac.jp',
    description='A ROS 2 node that subscribes to control commands and modifies them.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_control_node = my_control_node.my_control_node:main',  # スクリプトのパスを正しく指定
        ],
    },
    package_data={package_name: ['package.xml']},
    include_package_data=True,
)
