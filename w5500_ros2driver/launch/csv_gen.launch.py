from launch import LaunchDescription
from launch_ros.actions import Node

package_name = "w5500_ros2driver"

def generate_launch_description():
    return LaunchDescription([
        Node(
            package=package_name,
            executable="csv_gen",
            name="logger_1",
            namespace="sensor_1",
            parameters=[{"sensor_name": "sensor_1", "port": 5000}]
        ),
        Node(
            package=package_name,
            executable="csv_gen",
            name="logger_2",
            namespace="sensor_2",
            parameters=[{"sensor_name": "sensor_2", "port": 5001}]
        ),
        Node(
            package=package_name,
            executable="csv_gen",
            name="logger_3",
            namespace="sensor_3",
            parameters=[{"sensor_name": "sensor_3", "port": 5002}]
        ),
        Node(
            package=package_name,
            executable="csv_gen",
            name="logger_4",
            namespace="sensor_4",
            parameters=[{"sensor_name": "sensor_4", "port": 5003}]
        ),
    ])
