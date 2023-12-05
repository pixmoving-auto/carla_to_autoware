import launch

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='test_param1',
            default_value='Hello!'
        ),
        Node(
            package='carla_autoware_bridge',
            executable='test_pub_node',
            name='test_pub_node_start_by_xml',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'test_param': launch.substitutions.LaunchConfiguration('test_param1')
                },
            ],
            # remappings=[
            #     ('/input/pose', '/turtlesim1/turtle1/pose'),
            #     ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            # ]
        ),
    ])