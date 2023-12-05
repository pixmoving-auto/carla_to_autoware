import launch

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='topic_tools',
            executable='relay',
            name='sensing_camera_trafficLight_cameraInfo_relay',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {'input_topic': '/carla/base_link/rgb_front/camera_info'},
                {'output_topic': '/sensing/camera/traffic_light/camera_info'},
                {'type': 'sensor_msgs/msg/CameraInfo'},
                {'reliability': 'best_effort'},
            ],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='sensing_camera_trafficLight_image_raw_relay',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {'input_topic': '/carla/base_link/rgb_front/image'},
                {'output_topic': '/sensing/camera/traffic_light/image_raw'},
                {'type': 'sensor_msgs/msg/Image'},
                {'reliability': 'best_effort'},
            ],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='sensing_imu_tamagawa_imuRaw_relay',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {'input_topic': '/carla/base_link/imu'},
                {'output_topic': '/sensing/imu/tamagawa/imu_raw'},
                {'type': 'sensor_msgs/msg/Imu'},
                {'reliability': 'best_effort'},
            ],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='concatenated_pointcloud_unfilter_relay',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {'input_topic': '/carla/base_link/lidar'},
                {'output_topic': '/sensing/lidar/concatenated/pointcloud_unfilter'},
                {'type': 'sensor_msgs/msg/PointCloud2'},
                {'reliability': 'best_effort'},
            ],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='odometry_relay',
            output='screen',
            emulate_tty='True',
            on_exit=launch.actions.Shutdown(),
            parameters=[
                {'input_topic': '/carla/base_link/odometry'},
                {'output_topic': '/localization/kinematic_state'},
                {'type': 'nav_msg/msg/Odometry'},
                {'reliability': 'best_effort'},
            ],
        ),
    ])