import os
from glob import glob
from setuptools import setup

package_name = 'w5500_ros2driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Danny Gutiérrez Campos y Luis Diego Soto',
    maintainer_email='dannyjgc07@gmail.com y ls.sotochaves@gmail.com',
    description='ROS2 node package for OpenCoRoCo: w5500_ros2driver',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'force_publisher = w5500_ros2driver.w5500:main',
        ],
    },
)
