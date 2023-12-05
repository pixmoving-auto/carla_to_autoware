#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
ground truth localization. Publishes the following topics:
    /ground_truth/current_velocty (geometry_msgs::TwistStamped)
    /ground_truth/current_pose    (geometry_msgs::PoseStamped)
"""
import rclpy
import math
import tf2_ros


from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

class CarlaToAutowareLocalization(Node):

    def __init__(self):
        super().__init__('carla_to_autoware_localization')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/ground_truth/current_pose', queue_size=1)
        
        self.twist_pub = self.create_publisher(TwistStamped, '/ground_truth/current_velocity', queue_size=1)

        self.subscription = self.create_subscription(Odometry, '/carla/{}/odometry'.format(self.declare_parameter('role_name', 'ego_vehicle')), self.callback, 10)

    def callback(self, data):
        """
        callback odometry
        """

        # Transform
        br = tf2_ros.TransformBroadcaster()
        origin = data.pose.pose.position
        orientation = data.pose.pose.orientation
        br.sendTransform((origin.x, origin.y, origin.z),
                        (orientation.x, orientation.y, orientation.z, orientation.w),
                        Clock().now(),
                        '/base_link', '/map')

        # Pose
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose

        # Twist
        linear = data.twist.twist.linear
        angular = data.twist.twist.angular

        twist = TwistStamped()
        twist.header = data.header
        twist.twist.linear.x = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
        twist.twist.angular = angular

        self.pose_pub.publish(pose)
        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = CarlaToAutowareLocalization()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

