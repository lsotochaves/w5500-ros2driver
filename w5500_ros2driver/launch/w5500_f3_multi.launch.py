from launch import LaunchDescription
from launch_ros.actions import Node

package_name = "w5500_ros2driver"

def generate_launch_description():
    return LaunchDescription([
        # --- W5500 #1 on port 5000 ---
        Node(
            package=package_name,
            executable="force_publisher",
            name="forcestick_1",
            namespace="sensor_1",
            output="screen",
            parameters=[{
                "server_port": 5000,
                "use_udp": True
            }]
        ),

        # --- W5500 #2 on port 5001 ---
        Node(
            package=package_name,
            executable="force_publisher",
            name="forcestick_2",
            namespace="sensor_2",
            output="screen",
            parameters=[{
                "server_port": 5001,
                "use_udp": True
            }]
        ),

        # --- W5500 #3 on port 5002 ---
        Node(
            package=package_name,
            executable="force_publisher",
            name="forcestick_3",
            namespace="sensor_3",
            output="screen",
            parameters=[{
                "server_port": 5002,
                "use_udp": True
            }]
        ),

        # --- W5500 #4 on port 5003 ---
        Node(
            package=package_name,
            executable="force_publisher",
            name="forcestick_4",
            namespace="sensor_4",
            output="screen",
            parameters=[{
                "server_port": 5003,
                "use_udp": True
            }]
        ),

        # --- STM32F3 (Loadcell + IMU) on port 5004 ---
        Node(
            package=package_name,
            executable="force_publisher",
            name="forcestick_f3",
            namespace="sensor_f3",
            output="screen",
            parameters=[{
                "server_port": 5004,
                "use_udp": True,
                # Default values (can be overridden from CLI or setup menu)
                "opmode": 1,    # 0: voltage mode, 1: full force mode
                "sistema": 0,   # 0: 10 kg, 1: 200 kg
                "plane": 0      # 0: front, 1: left, 2: back, 3: right
            }]
        ),
    ])
