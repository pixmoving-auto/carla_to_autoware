import rclpy
import time
from std_msgs.msg import Int32
from rclpy.node import Node

class Pub(Node):
    def __init__(self):
        super().__init__('test_pub')
        self.msg = Int32()
        self.msg.data = int(1)
        self.pub = self.create_publisher(Int32, '/test_topic', 1)
        time.sleep(2)
        self.pub.publish(self.msg)
        print("publish", self.msg.data)

        # self.msg.data += 1
        # self.declare_parameter('test_param', 'world')
        # param = self.get_parameter('test_param')
        # print("print param", param.get_parameter_value().string_value)

    # def publish(self):
    #     self.pub.publish(self.msg)
    #     print("publish", self.msg.data)

        
def main(args=None):
    """
    main loop
    """
    rclpy.init(args=args)

    node = Pub()

    # while True:
    #     node.publish()
    #     time.sleep(1)

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()