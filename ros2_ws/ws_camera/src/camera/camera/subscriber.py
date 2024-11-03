# camera/subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # 変更点


class ArraySubscriber(Node):
    def __init__(self):
        super().__init__('array_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,  # 型を変更
            'array_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info('Array Subscriber Node started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ArraySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
