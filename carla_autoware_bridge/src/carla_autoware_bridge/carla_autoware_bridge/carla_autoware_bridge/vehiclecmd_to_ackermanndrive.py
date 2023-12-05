#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive autoware_msgs::VehicleCmd and publish ackermann_msgs::AckermannDrive
"""
import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from autoware_vehicle_msgs.msg import VehicleCommand
from std_msgs.msg import Bool


class VehiclecmdToAckermanndrive(Node):

    def __init__(self):
        super().__init__('twist_to_ackermanndrive')
        
        role_name = self.declare_parameter('role_name', 'base_link')
        self.pub = self.create_publisher(AckermannDrive, '/carla/{}/ackermann_cmd'.format(role_name), queue_size=1)

        self.create_subscription(VehicleCommand, '/control/vehicle_cmd', self.callback, queue_size=1)

    def callback(data, self):
        msg = AckermannDrive()
        msg.acceleration = data.control.acceleration
        msg.speed = data.control.velocity
        if(data.shift.data == 2):
            msg.speed = -msg.speed
            msg.acceleration = -msg.acceleration
        msg.steering_angle = data.control.steering_angle * 2
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = VehiclecmdToAckermanndrive()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
