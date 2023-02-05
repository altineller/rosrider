from setuptools import setup

package_name = 'rosrider_services'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml'])
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
            'service = rosrider_services.rosrider_service:main'
        ],
    },
)
