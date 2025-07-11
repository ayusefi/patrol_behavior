from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Path to bcr_bot launch files
    bcr_bot_launch_dir = os.path.join(
        get_package_share_directory('bcr_bot'), 'launch')

    # Gazebo simulation
    ign_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bcr_bot_launch_dir, 'ign.launch.py')
        )
    )

    # Nav2 stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bcr_bot_launch_dir, 'nav2.launch.py')
        )
    )

    # Patrol behavior node (delayed start)
    patrol_node = Node(
        package='patrol_behavior',
        executable='patrol_node',
        name='patrol_behavior_node',
        output='screen'
    )
    delayed_patrol_node = TimerAction(
        period=10.0,  # seconds, adjust as needed
        actions=[patrol_node]
    )

    ld.add_action(ign_launch)
    ld.add_action(nav2_launch)
    ld.add_action(delayed_patrol_node)

    return ld
