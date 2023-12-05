#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
"""
ground truth detections. Publishes the following topics:
    /ground_truth/objects  (autoware_msgs::DetectedObjectArray)
"""
import rclpy
import uuid
from rclpy.node import Node

from autoware_perception_msgs.msg import DynamicObjectArray, DynamicObject, Semantic, Shape
from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker, MarkerArray

class CarlaToAutowareDetectedObjects(Node):

    def __init__(self):
        super().__init__('carla_to_autoware_detected_objects')

        self.pub = self.create_publisher(DynamicObjectArray, '/perception/object_recognition/objects', queue_size=1)

        self.subscription = self.create_subscription(ObjectArray, '/carla/{}/objects'.format(self.declare_parameter('role_name', 'base_link')), self.callback, 10)

    def callback(self, data):
        """
        callback for current objects
        """
        dynamic_objects_msg = DynamicObjectArray()
        dynamic_objects_msg.header = data.header

        for obj in data.objects:
            dynamic_object_msg = DynamicObject()
            uuid_temp = uuid.uuid3(uuid.NAMESPACE_DNS, str(obj.id)).bytes
            dynamic_object_msg.id.uuid = uuid_temp
            if obj.classification == Object.CLASSIFICATION_MOTORCYCLE:
                dynamic_object_msg.semantic.type = Semantic.MOTORBIKE
            elif obj.classification == Object.CLASSIFICATION_BIKE:
                dynamic_object_msg.semantic.type = Semantic.BICYCLE
            elif obj.classification == Object.CLASSIFICATION_CAR:
                dynamic_object_msg.semantic.type = Semantic.CAR
            elif obj.classification == Object.CLASSIFICATION_PEDESTRIAN:
                dynamic_object_msg.semantic.type = Semantic.PEDESTRIAN
            elif obj.classification == Object.CLASSIFICATION_TRUCK:
                dynamic_object_msg.semantic.type = Semantic.TRUCK
            elif obj.classification == Object.CLASSIFICATION_UNKNOWN_SMALL:
                dynamic_object_msg.semantic.type = Semantic.ANIMAL
            elif obj.classification == Object.CLASSIFICATION_UNKNOWN_MEDIUM:
                dynamic_object_msg.semantic.type = Semantic.ANIMAL
            else:
                dynamic_object_msg.semantic.type = Semantic.UNKNOWN
            dynamic_object_msg.semantic.confidence = obj.classification_certainty

            dynamic_object_msg.state.pose_covariance.pose = obj.pose
            dynamic_object_msg.state.orientation_reliable = True
            dynamic_object_msg.state.twist_covariance.twist = obj.twist
            dynamic_object_msg.state.twist_reliable = True
            dynamic_object_msg.state.acceleration_covariance.accel = obj.accel
            dynamic_object_msg.state.acceleration_reliable = True

            if obj.shape.type == SolidPrimitive.BOX:
                dynamic_object_msg.shape.type = Shape.BOUNDING_BOX
            elif obj.shape.type == SolidPrimitive.CYLINDER:
                dynamic_object_msg.shape.type = Shape.CYLINDER
            else:
                dynamic_object_msg.shape.type = Shape.POLYGON
            dynamic_object_msg.shape.dimensions.x = obj.shape.dimensions[0]
            dynamic_object_msg.shape.dimensions.y = obj.shape.dimensions[1]
            dynamic_object_msg.shape.dimensions.z = obj.shape.dimensions[2]
            dynamic_object_msg.shape.footprint = obj.polygon
            dynamic_objects_msg.objects.append(dynamic_object_msg)

        self.pub.publish(dynamic_objects_msg)

def main(args=None):
    rclpy.init(args=args)

    node = CarlaToAutowareDetectedObjects()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()