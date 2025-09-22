from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ebimu_pkg = Node(
        package='ebimu_pkg',
        executable='imu_pub',
        name='imu_pub',
        output='screen'
    )
    
    return LaunchDescription([
        ebimu_pkg
    ])
