import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'handsfree_ros2_imu'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='HandsFree',
    maintainer_email='hands_free@126.com',
    description='ROS2 driver for HandsFree IMU sensors (A9, B6, B9)',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hfi_a9_node = handsfree_ros2_imu.hfi_a9_node:main',
            'hfi_b6_node = handsfree_ros2_imu.hfi_b6_node:main',
            'hfi_b9_node = handsfree_ros2_imu.hfi_b9_node:main',
            'get_imu_rpy = handsfree_ros2_imu.get_imu_rpy:main',
        ],
    },
)

