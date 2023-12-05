import rclpy
from rclpy.node import Node
from sensor_msgs.msg import (
   Imu,
   PointCloud2,
   NavSatFix,
   CameraInfo,
   Image
)
from carla_msgs.msg import CarlaStatus
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from carla_msgs.msg import (
    CarlaStatus,
    CarlaEgoVehicleInfo,
    CarlaEgoVehicleStatus,
    CarlaLaneInvasionEvent,
    CarlaCollisionEvent
)
from derived_object_msgs.msg import ObjectArray

class Sub(Node):
    def __init__(self):
        super().__init__('test_sub')
        print("hello")
        
        # self.imu_subscription = self.create_subscription(Imu, '/carla/ego_vehicle/imu', self.Callback, qos_profile=10)
        # self.carla_status_subscription = self.create_subscription(CarlaStatus, "/carla/status", self.Callback, qos_profile=10)
        # self.clock_subscription = self.create_subscription(Clock, 'clock', self.Callback, 10)
        # self.lane_invasion_subscription = self.create_subscription(CarlaLaneInvasionEvent, '/carla/ego_vehicle/lane_invasion', self.Callback, 10)
        # self.collision_subscription = self.create_subscription(CarlaCollisionEvent, '/carla/ego_vehicle/collision', self.Callback, 10)
        # self.semantic_lidar_subscription = self.create_subscription(PointCloud2, '/carla/ego_vehicle/semantic_lidar', self.Callback, 10)
        # self.lidar_subscription = self.create_subscription(PointCloud2, '/carla/ego_vehicle/lidar', self.Callback, 10)
        # self.gnss_subscription = self.create_subscription(NavSatFix, '/carla/ego_vehicle/gnss', self.Callback, 10)
        # self.object_subscription = self.create_subscription(ObjectArray, '/carla/objects', self.Callback, 10)
        # self.rgb_camera_info_subscription = self.create_subscription(CameraInfo, '/carla/ego_vehicle/rgb_view/camera_info', self.Callback, 10)
        # self.rgb_camera_image_subscription = self.create_subscription(Image, '/carla/ego_vehicle/rgb_view/image', self.Callback, 10)
        # self.rgb_camera_image_subscription = self.create_subscription(PointCloud2, '/carla/ego_vehicle/dvs_front/events', self.Callback, 10)
        # self.odometry_subscription = self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.Callback, 10)
        # self.object_subscription = self.create_subscription(ObjectArray, '/carla/ego_vehicle/objects', self.Callback, 10)

## /carla/ego_vehicle/rgb_front
## /carla/ego_vehicle/semantic_segmentation_front
## /carla/ego_vehicle/depth_front
## /carla/ego_vehicle/dvs_front

    def Callback(self, data):
        print("Data = ", data)


def main(args=None):
    rclpy.init(args=args)

    node = Sub()

    rclpy.spin(node)

    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()