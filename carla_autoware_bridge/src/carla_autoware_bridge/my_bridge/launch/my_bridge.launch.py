import launch
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'my_bridge'), 'relay.launch.py')
            ),
        ),
        Node(
            package='my_bridge',
            executable='carla_to_autoware_localization_node',
            name='carla_to_autoware_localization_node',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
        ),
        Node(
            package='my_bridge',
            executable='carla_to_autoware_vehicle_status_node',
            name='carla_to_autoware_vehicle_status_node',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
        ),
        Node(
            package='my_bridge',
            executable='odometry_to_posestamped_node',
            name='odometry_to_posestamped_node',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
        ),
        Node(
            package='my_bridge',
            executable='vehiclecmd_to_ackermanndrive',
            name='vehiclecmd_to_ackermanndrive',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
        ),
    ])