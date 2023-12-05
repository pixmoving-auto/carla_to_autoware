import rclpy
import std_msgs.msg
from rclpy.node import Node

class Sub(Node):
    def __init__(self):
        super().__init__('test_sub')

        # self.create_subscription(std_msgs.msg.Int32, '/test_topic', self.callback, 10)
        self.create_subscription(std_msgs.msg.Int32, '/test_topic_relay', self.callback, 10)
        

    def callback(self, data):
        print(data)

        
def main(args=None):
    """
    main loop
    """
    rclpy.init(args=args)

    node = Sub()

    rclpy.spin(node)

    node.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()