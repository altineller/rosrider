from setuptools import setup
import os
from glob import glob

package_name = 'rosrider_services'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Can Altineller',
    maintainer_email='altineller@gmail.com',
    description='ROS2 services for controlling ROSRider board',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drivemode = rosrider_services.drivemode_service:main',
            'led = rosrider_services.led_service:main',
            'pidtune = rosrider_services.pidtune_service:main',
            'setfloat = rosrider_services.setfloat_service:main',
            'setint = rosrider_services.setint_service:main',
            'setrtc = rosrider_services.setrtc_service:main',
            'sysctl = rosrider_services.sysctl_service:main'
        ],
    },
)
