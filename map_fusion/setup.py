from setuptools import setup
import os
from glob import glob

package_name = 'map_fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],  # 移除 opencv-python
    zip_safe=True,
    maintainer='alex',
    maintainer_email='717863696@qq.com',
    description='ORB feature-based map fusion node for combining LRF-based and pseudo-laser-based occupancy grids',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_fusion_node = map_fusion.map_fusion_node:main',
        ],
    },
)