import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3 # Neato control messages

class Publisher(Node):
    def __init__(self):
        super().__init__("publisher_node")

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.send_msg)
        self.publisher = self.create_publisher(String, "my_topic", 10) # defines the script as a ros publisher\

    def send_msg(self):
        msg = String(data = "hello")
        #msg = Twist(lin_vel = Vector3(x=0.0, y=0.0, z=0.0), ang_vel = Vector3(x=0.0, y=0.0, z=0.0))
        
        self.publisher.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
