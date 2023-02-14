from setuptools import setup
import os
from glob import glob

package_name = 'rosrider_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Can Altineller',
    maintainer_email='altineller@gmail.com',
    description='ROS2 differential drive and odometry node for ROSRider board',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosrider_node = rosrider_node.rosrider_node:main',
        ],
    },
)
