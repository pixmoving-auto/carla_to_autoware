#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
receive :CarlaEgoVehicleStatus and publishes autoware_msgs::VehicleStatus
"""
import rclpy
import math

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaEgoVehicleStatus

from autoware_vehicle_msgs.msg import Steering, ShiftStamped, Shift, TurnSignal, ControlMode

vehicle_info = None
manual_override = None

class CarlaToAutowareVehicleStatus(Node):

    def __init__(self):
        super().__init__('carla_to_autoware_vehicle_status')
        
        self.pub_steer = self.create_publisher(Steering, '/vehicle/status/steering', queue_size=1)
        self.pub_shift = self.create_publisher(ShiftStamped, '/vehicle/status/shift', queue_size=1)
        self.pub_turn_signal = self.create_publisher(TurnSignal, '/vehicle/status/turn_signal', queue_size=1)
        self.pub_control_mode = self.create_publisher(ControlMode, '/vehicle/status/control_mode', queue_size=1)
        self.pub_twist_stamped = self.create_publisher(TwistStamped, '/vehicle/status/twist', queue_size=1)
        self.pub_velocity = self.create_publisher(Float32, '/vehicle/status/velocity', queue_size=1)

        role_name = self.declare_parameter('role_name', 'base_link')
        self.create_subscription(CarlaEgoVehicleStatus, '/carla/{}/vehicle_status'.format(role_name), self.vehicle_status_callback, 10)
        self.create_subscription(CarlaEgoVehicleInfo, '/carla/{}/vehicle_info'.format(role_name), self.vehicle_info_callback, 10)
        self.create_subscription(Odometry, '/carla/{}/odometry'.format(role_name), self.odom_callback, 10)
        self.create_subscription(Bool, '/carla/{}/vehicle_control_manual_override'.format(role_name), self.manual_override_callback, 10)

    def vehicle_info_callback(self, data):
        """
        callback for vehicle info
        """
        global vehicle_info
        vehicle_info = data

    def odom_callback(self, data):
        """
        callback for vehicle odometry
        """
        twist = TwistStamped()
        twist.header = data.header
        twist.twist = data.twist.twist
        self.pub_twist_stamped.publish(twist)

    def manual_override_callback(self, data):
        global manual_override
        manual_override = data



    def vehicle_status_callback(self, data):
        """
        callback for vehicle status
        """
        if vehicle_info is None:
            return
        if manual_override is None:
            return
        steer = Steering()
        shift = ShiftStamped()
        control_mode = ControlMode()

        velocity = Float32()
        steer.header = data.header
        shift.header = data.header

        # calculate max steering angle
        max_steering_angle = math.radians(70)
        # get max steering angle (use smallest non-zero value of all wheels)
        for wheel in vehicle_info.wheels:
            if wheel.max_steer_angle:
                if wheel.max_steer_angle and wheel.max_steer_angle < max_steering_angle:
                    max_steering_angle = wheel.max_steer_angle
        steer.data = data.control.steer * max_steering_angle

        velocity = data.velocity

        if data.control.reverse:
            shift.shift = Shift.REVERSE
        else:
            shift.shift = Shift.DRIVE
        
        control_mode.header = data.header
        control_mode = ControlMode()
        if manual_override.data:
            control_mode.data = ControlMode.MANUAL
        else:
            control_mode.data = ControlMode.AUTO

        self.pub_control_mode.publish(control_mode)
        self.pub_steer.publish(steer)
        self.pub_shift.publish(shift)
        self.pub_velocity.publish(velocity)

def main(args=None):
    rclpy.init(args=args)

    node = CarlaToAutowareVehicleStatus()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()