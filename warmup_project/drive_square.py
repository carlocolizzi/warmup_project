import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

class driveSquare(Node):
    def __init__(self):
        super().__init__('drive_square')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.run_loop()        
    
    def run_loop(self):
        while True:
            msg = Twist()

            msg.linear.x = 1.0
            msg.angular.z = 0.0

            self.vel_pub.publish(msg)
            time.sleep(3)

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.vel_pub.publish(msg)
            time.sleep(0.5)

            msg.angular.z = 1.0
            msg.linear.x = 0.0

            self.vel_pub.publish(msg)
            time.sleep(2.06)

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.vel_pub.publish(msg)
            time.sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    node = driveSquare()
    rclpy.spin(node)
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()