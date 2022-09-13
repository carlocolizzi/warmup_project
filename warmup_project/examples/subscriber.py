import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 # Neato control messages

class Subscriber(Node):

    def __init__(self):
        super().__init__("subscriber_node")
        self.subscriber = self.create_subscription(String, 'my_topic', self.process_input, 10)

    def process_input(self, msg):
        print(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()