import tty
import select
import sys
import termios
import rclpy
from rclpy.node import Node
from neato2_interfaces.msg import Bump
from geometry_msgs.msg import Twist

class teleop(Node):
    def __init__(self):
        super().__init__('teleop')
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.run_loop()        
    
    def run_loop(self):
        key = None
        while key != '\x03':
            msg = Twist()

            key = getKey()
            print(key)

            if key == 'w':
                msg.linear.x = 1.0
            elif key == 's':
                msg.linear.x = -1.0
            elif key == 'a':
                msg.angular.z = 1.0
            elif key == 'd':
                msg.angular.z = -1.0
            self.vel_pub.publish(msg)
            

def getKey():
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    node = teleop()
    rclpy.spin(node)
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()