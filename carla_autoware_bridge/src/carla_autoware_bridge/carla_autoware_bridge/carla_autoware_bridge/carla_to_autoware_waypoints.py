#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive a path from carla_ros_waypoint_publisher and convert it to autoware
"""
import rclpy
from rclpy.node import Node
from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import Lane
from autoware_msgs.msg import Waypoint
from nav_msgs.msg import Path

class CarlaToAutowareWaypoints(Node):

    def __init__(self):
        super().__init__('carla_to_autoware_waypoints')
        
        self.pub = self.create_publisher(LaneArray, '/lane_waypoints_array', queue_size=1, latch=True)

        self.create_subscription(Path, '/carla/{}/waypoints'.format(self.declare_parameter('role_name', 'ego_vehicle')), self.path_callback, 10)

    def path_callback(self, data):
        """
        callback for path. Convert it to Autoware LaneArray and publish it
        """
        msg = LaneArray()
        lane = Lane()
        lane.header = data.header
        msg.lanes.append(lane)
        for pose in data.poses:
            waypoint = Waypoint()
            waypoint.pose = pose
            msg.lanes[0].waypoints.append(waypoint)

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = CarlaToAutowareWaypoints()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
