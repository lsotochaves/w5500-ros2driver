from launch import LaunchDescription
from launch_ros.actions import Node

package_name = "w5500_ros2driver"

def generate_launch_description():
    return LaunchDescription([
        # W5500 #1 on port 5000
        Node(
            package=package_name,  # Replace with your package name
            executable="force_publisher",  # The name you installed with setup.py entry_point
            name="forcestick_1",
            namespace="sensor_1",
            output="screen",
            parameters=[{
                "server_port": 5000,
                "csv_prefix": "output_w5500_1"
            }]
        ),

        # W5500 #2 on port 5001
        Node(
            package=package_name,
            executable="force_publisher",
            name="forcestick_2",
            namespace="sensor_2",
            output="screen",
            parameters=[{
                "server_port": 5001,
                "csv_prefix": "output_w5500_2"
            }]
        ),

        # W5500 #3 on port 5002
        Node(
            package=package_name,
            executable="force_publisher",
            name="forcestick_3",
            namespace="sensor_3",
            output="screen",
            parameters=[{
                "server_port": 5002,
                "csv_prefix": "output_w5500_3"
            }]
        ),

        # W5500 #4 on port 5003
        Node(
            package=package_name,
            executable="force_publisher",
            name="forcestick_4",
            namespace="sensor_4",
            output="screen",
            parameters=[{
                "server_port": 5003,
                "csv_prefix": "output_w5500_4"
            }]
        ),
    ])
