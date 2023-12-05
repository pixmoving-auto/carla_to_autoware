#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive nav_msgs::Odometry and publish geometry_msgs::PoseStamped
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from autoware_auto_vehicle_msgs.msg import VelocityReport

class OdometryToPosestamped(Node):

    def __init__(self):
        super().__init__('convert_odometry_to_pose')
        
        self.pose_pub = self.create_publisher(PoseStamped, '/sensing/gnss/pose', 1)
        self.pose_with_covariance_pub = self.create_publisher(PoseWithCovarianceStamped, '/sensing/gnss/pose_with_covariance', 1)
        self.pub_velocity_report = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 1)
        self.create_subscription(Odometry, '/carla/base_link/odometry', self.callback, 10)


    def callback(self, data):
        # print("convert carla_odometry to senser pose_info")
        pose = PoseStamped()
        pose_with_covariance = PoseWithCovarianceStamped()
        velocity_report = VelocityReport()

        pose.header = data.header
        pose.pose = data.pose.pose

        pose_with_covariance.header = data.header
        pose_with_covariance.pose.pose = data.pose.pose
        pose_with_covariance.pose.covariance = data.pose.covariance

        velocity_report.header = data.header
        if abs(data.twist.twist.linear.x) < 0.01:
            data.twist.twist.linear.x = 0.0005
        velocity_report.longitudinal_velocity = data.twist.twist.linear.x
        velocity_report.lateral_velocity = 0.0
        velocity_report.heading_rate = 0.0

        self.pose_pub.publish(pose)
        self.pose_with_covariance_pub.publish(pose_with_covariance)
        self.pub_velocity_report.publish(velocity_report)
    
def main(args=None):
    rclpy.init(args=args)

    node = OdometryToPosestamped()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
