from  launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='digitaltwin',
            node_namespace='',
            node_executable='asset.py',
            node_name='uav_asset',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='digitaltwin',
            node_namespace='',
            node_executable='twin.py',
            node_name='uav_twin',
            output='screen',
            emulate_tty=True,
        ),
    ])
