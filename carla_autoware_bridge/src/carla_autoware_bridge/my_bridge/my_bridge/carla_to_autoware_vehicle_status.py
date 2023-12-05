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

from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import TwistStamped, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool, UInt8

from rclpy.node import Node
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_msgs.msg import CarlaEgoVehicleStatus

from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport, SteeringReport, VelocityReport


vehicle_info = None
manual_override = None

class CarlaToAutowareVehicleStatus(Node):

    def __init__(self):
        super().__init__('carla_to_autoware_vehicle_status')
        
        self.pub_control_mode_report = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 1)
        self.pub_gear_report = self.create_publisher(GearReport, '/vehicle/status/gear_status', 1)
        self.pub_steer_report = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 1)
        self.pub_acceleration_report = self.create_publisher(AccelWithCovarianceStamped, '/localization/acceleration', 1)

        self.create_subscription(CarlaEgoVehicleInfo, '/carla/base_link/vehicle_info', self.vehicle_info_callback, 10)
        self.create_subscription(CarlaEgoVehicleStatus, '/carla/base_link/vehicle_status', self.vehicle_status_callback, 10)
        latching_qos = QoSProfile(depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/carla/base_link/vehicle_control_manual_override', self.manual_override_callback, qos_profile=latching_qos)
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.manual_override = Bool()

    def vehicle_info_callback(self, data):
        """
        callback for vehicle info
        """
        self.vehicle_info = data

    def manual_override_callback(self, data):
        self.manual_override = data

    def vehicle_status_callback(self, data):
        """
        callback for vehicle status
        """
        # print("convert carla_vehicle_status to autoware_vehicle_status")
        if self.vehicle_info is None:
            return
        if self.manual_override is None:
            return
        
        control_mode_report = ControlModeReport()
        gear_report = GearReport()
        steer_report = SteeringReport()
        
        acceleration_report = AccelWithCovarianceStamped()

        control_mode_report.stamp = data.header.stamp
        # module ControlModeReport_Constants {
        #     const uint8 NO_COMMAND = 0;
        #     const uint8 AUTONOMOUS = 1;
        #     const uint8 AUTONOMOUS_STEER_ONLY = 2;
        #     const uint8 AUTONOMOUS_VELOCITY_ONLY = 3;
        #     const uint8 MANUAL = 4;
        #     const uint8 DISENGAGED = 5;
        #     const uint8 NOT_READY = 6;
        # }
        if self.manual_override.data:
            control_mode_report.mode = 4
        else:
            control_mode_report.mode = 1

        gear_report.stamp = data.header.stamp
        while data.control.gear < 0:
            data.control.gear += 256
        while data.control.gear >=256:
            data.control.gear -=256
        gear_report.report = data.control.gear

        steer_report.stamp = data.header.stamp
        # calculate max steering angle
        max_steering_angle = math.radians(70)
        # get max steering angle (use smallest non-zero value of all wheels)
        for wheel in self.vehicle_info.wheels:
            if wheel.max_steer_angle:
                if wheel.max_steer_angle and wheel.max_steer_angle < max_steering_angle:
                    max_steering_angle = wheel.max_steer_angle
        steer_report.steering_tire_angle = data.control.steer * max_steering_angle

        

        acceleration_report.header = data.header
        acceleration_report.accel.accel = data.acceleration

        self.pub_control_mode_report.publish(control_mode_report)
        self.pub_gear_report.publish(gear_report)
        self.pub_steer_report.publish(steer_report)
        
        # self.pub_acceleration_report.publish(acceleration_report)

def main(args=None):
    rclpy.init(args=args)

    node = CarlaToAutowareVehicleStatus()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()