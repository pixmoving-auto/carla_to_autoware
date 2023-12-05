import launch

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='relay',
            name='test_topic_relay',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {
                    'input_topic': '/test_topic'
                },
                {
                    'output_topic': '/test_topic_relay'
                },
                {
                    'type': 'Int32'
                },
                {
                    'reliability': 'best_effort'
                },
            ],
        ),
    ])