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
from nav_msgs.msg import Odometry

class OdometryToPosestamped(Node):

    def __init__(self):
        super().__init__('convert_odometry_to_pose')
        
        self.pub = self.create_publisher(PoseStamped, '/current_pose', queue_size=1)

        self.create_subscription(Odometry, '/carla/{}/odometry'.format(self.declare_parameter('role_name', 'base_link')), self.callback, 10)

    def callback(data, self):
        pose = PoseStamped()
        pose.header = data.header
        pose.pose = data.pose.pose
        self.pub.publish(pose)
    
def main(args=None):
    rclpy.init(args=args)

    node = OdometryToPosestamped()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
