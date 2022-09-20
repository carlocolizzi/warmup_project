## Carlo Colizzi, Ben Grant
# 09.19.2022

# import ROS2 libraries
from turtle import heading
import 
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose, Vector3Stamped # Neato control messages
from rclpy.qos import qos_profile_sensor_data

# import python libraries
import time
from simple_pid import PID
import math
import numpy as np

class ObstacleAvoider(Node):

    def __init__(self):
        # run superclass constructor
        super().__init__("laser_scan")

        # create subscribers and publishers
        self.create_subscription(LaserScan, 'scan', self.process_scan,\
                                 qos_profile=qos_profile_sensor_data)      # lidar sub
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)  # odometry sub
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)         # velocity pub

        # define state members
        self.pid = PID(3, 0.5, 5, setpoint=0)
        self.target = [10.0, 1.0]       # (m) waypoint to follow
        self.position_x = 0.0
        self.position_y = 0.0
        self.heading = 0.0
        self.distances = np.ones(360)

        #self.run_loop()

    def process_odom(self, msg):
        position = msg.pose.pose.position
        self.position_x = position.x
        self.position_y = position.y

        # debug
        #print_same_line("pos | "\
        #     +f"x: {round(self.position_x, 2)} | "\
         #    +f"y: {round(self.position_y, 2)} | ")

    def process_scan(self, msg):

        # use laserscan to find objects in local polar coords
        
        # update state members from lidar subscription data
        for index in range(0,len(msg.ranges)):
            if math.isinf(msg.ranges[index]):
                msg.ranges[index] = 0.0
        nonzero_indices = np.flatnonzero(msg.ranges)
        remapped_indices = []
        for index in range(0,len(nonzero_indices)):
            remapped_indices.append(remap(nonzero_indices[index]))
        
        self.heading = np.mean(remapped_indices)
        self.distances = msg.ranges
        self.run_loop()
        
    def run_loop(self):
        # neato's pos in global coords
        position = [self.position_x, self.position_y]
        # position of obstacle in local polar coords
        if self.heading > 0:
            obstacles = [self.heading, self.distances[int(remap2(self.heading))]]
            # pos of obstacle in local cart coords
            obstacles_cart = pol2cart(obstacles[1], math.radians(obstacles[0]))
            distance_to_obstacle = math.sqrt(obstacles_cart[0]**2 + obstacles_cart[1]**2)
            obstacle_weight = 1+ 1/distance_to_obstacle


            # sum of attraction and repulsion vectors
            vector = np.subtract(np.subtract(self.target, position), np.multiply(obstacles_cart,obstacle_weight))
        else:
            vector = np.subtract(self.target, position)
        # heading to make the neato follow
        angle = math.atan2(vector[1], vector[0])
        #print(angle)
        # send velocity cmd to neato
        self.drive_to_vector(angle)


    def drive_to_vector(self, angle):
        msg = Twist()
        msg.linear.x = 1.0
        self.pid = PID(setpoint = angle)
        if angle < 0:

            msg.angular.z = self.pid(0)
        elif angle > 0:
            msg.angular.z = self.pid(0)
        else:
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            time.sleep(0.1)
            msg.linear.x = 1.0

        self.vel_pub.publish(msg)
        
def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

def remap(number):
        if number <= 180:
            result = number
        else:
            result = number - 360
        return result

def remap2(number):
        if number >= 0:
            result = number
        else:
            result = number + 360
        return result

def print_same_line(line):
    print(line, end = "")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
