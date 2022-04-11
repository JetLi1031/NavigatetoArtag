import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = os.path.join(get_package_share_directory('robotnavigate'), 'config', 'robotconfig.yaml')
    return LaunchDescription([
        Node(
            package='robotnavigate',
            executable='ar2tf.py',
            name='ar2tf',
            parameters = [params],
            output = 'screen'
        ),
        Node(
            package='robotnavigate',
            executable='batterypub',
            name='sim',
            parameters = [params],
            output = 'screen'
        ),
        Node(
            package='robotnavigate',
            executable='docking',
            name='todock',
            parameters = [params],
            output = 'screen'
        )
    ])
