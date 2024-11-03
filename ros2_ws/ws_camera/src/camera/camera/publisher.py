# camera/publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from . import get_object_position

class ArrayPublisher(Node):
    def __init__(self):
        super().__init__('array_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'array_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_array)
        self.get_logger().info('Array Publisher Node started.')

    def publish_array(self):
        msg = Float32MultiArray()  # 型を変更
        msg.data = get_object_position.get_position()  # Float型の配列に変更
        msg.data=[1.1,2.2,3.3]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ArrayPublisher()
    try:
    
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
