import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vehicle_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加 launch 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # 添加 description (URDF) 文件
        (os.path.join('share', package_name, 'description'), glob('description/*.urdf')),
        # 添加 config (YAML) 文件
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krtrobot',
    maintainer_email='qiuyu@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
