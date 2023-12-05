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
from std_msgs.msg import Bool, Int32
from carla_msgs.msg import CarlaEgoVehicleControl
from autoware_auto_control_msgs.msg import AckermannControlCommand

class VehiclecmdToAckermanndrive(Node):

    def __init__(self):
        super().__init__('twist_to_ackermanndrive')
        
        self.pub = self.create_publisher(AckermannDrive, '/carla/base_link/ackermann_cmd', 1)

        self.create_subscription(AckermannControlCommand, '/control/command/control_cmd', self.callback, 10)

    def callback(self, data):
        # print("convert control_cmd to carla_ackermann_cmd")
        msg = AckermannDrive()

        msg.steering_angle = data.lateral.steering_tire_angle
        msg.steering_angle_velocity = data.lateral.steering_tire_rotation_rate

        msg.speed = data.longitudinal.speed
        msg.acceleration = data.longitudinal.acceleration
        msg.jerk = data.longitudinal.jerk

        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    node = VehiclecmdToAckermanndrive()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
