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
        Node(
            package='rqt_gui',
            node_namespace='',
            node_executable='rqt_gui',
            node_name='rqt_gui',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='digitaltwin',
            node_namespace='',
            node_executable='logger.py',
            node_name='uav_logger',
            output='screen',
            emulate_tty=True,
        ),
    ])
