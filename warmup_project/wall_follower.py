
from turtle import distance
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3 # Neato control messages
from rclpy.qos import qos_profile_sensor_data

class Subscriber(Node):

    def __init__(self):
        super().__init__("laser_scan")
        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)

        # members
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.target_distance = 1.0
        self.previous_distance = 0.0

        
        self.create_timer(0.1, self.run_loop)

    
    def process_scan(self, msg):
        self.distance_to_wall = msg.ranges[270]
        self.previous_distance = self.distance_to_wall
        self.distance_to_obstacle = msg.ranges[0]
        # print("right", msg.ranges[270], "front", msg.ranges[0])
        print("dist_obstacle", self.distance_to_obstacle, "dist wall", self.distance_to_wall)

    def run_loop(self):
        msg = Twist()
        # check that robot is not crashing
        if self.distance_to_obstacle != 0.1:
            msg.linear.x = 1.
            
        msg.angular.z = 0.0
        
        if (self.distance_to_obstacle < self.target_distance) and (self.distance_to_obstacle >= self.previous_distance):
            # turn right (towards wall)
            # print("turning right")
            msg.angular.z = -1.0
        elif (self.distance_to_obstacle > self.target_distance) and (self.distance_to_obstacle <= self.previous_distance):
            # turn left  (away from wall)
            # print("turning left")
            msg.angular.z = 1.0
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
