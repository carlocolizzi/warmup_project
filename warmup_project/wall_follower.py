import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3 # Neato control messages

class Subscriber(Node):

    def __init__(self):
        super().__init__("laser_scan")
        self.create_subscription(LaserScan, 'scan', self.process_scan, 10)

    def process_scan(self, msg):
        if msg.ranges[0] != 0.0:
            self.distance_to_obstacle = msg.ranges[0]
        print(msg.ranges)

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    https://prod.liveshare.vsengsaas.visualstudio.com/join?1CB69895CF4FC8ACCE9C616C2FD9AF41B9CD