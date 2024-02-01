from setuptools import setup

package_name = 'forcestick_ros2driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Chen Wu',
    maintainer_email='hongwenchen.k.c.w.90@gmail.com',
    description='ROS2 node package for OpenCoRoCo: forcestick_ros2driver',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'force_publisher = forcestick_ros2driver.forcestick:main',
        ],
    },
)
