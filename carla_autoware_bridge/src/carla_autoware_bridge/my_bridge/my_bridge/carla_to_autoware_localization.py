#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
ground truth localization. Publishes the following topics:
    /awsim/ground_truth/vehicle/pose (geometry_msgs::PoseStamped)
"""
import rclpy
import tf2_ros

from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class CarlaToAutowareLocalization(Node):

    def __init__(self):
        super().__init__('carla_to_autoware_localization')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/awsim/ground_truth/vehicle/pose', 1)

        self.create_subscription(Odometry, '/carla/base_link/odometry', self.callback, 10)

    def callback(self, data):
        """
        callback odometry
        """
        # print("convert carla_odometry to ground_truth_pose")
        # Pose
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose

        # self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)

    node = CarlaToAutowareLocalization()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

