
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist # Neato control messages
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import math

class Subscriber(Node):

    def __init__(self):
        # run superclass constructor
        super().__init__("laser_scan")

        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)   # lidar sub
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)      # velocity pub
        # self.create_timer(0.1, self.run_loop)                           # loop timer
        
        # state variable members
        self.target_distance = 1.0
        self.previous_distance = None
        self.dist_right = 1.0
        self.dist_front = 1.0
        self.error = 0
    
    def process_scan(self, msg):
        # update state members from lidar subscription data
        self.dist_front = msg.ranges[0]

        self.previous_distance = self.dist_front

        self.dist_300 = msg.ranges[300]
        self.dist_270 = msg.ranges[270]
        
        self.calculate_angle()

        self.run_loop()

        # debug
        print(f"d right {round(self.dist_right, 3)}, d front {round(self.dist_front, 3)}", end="")

    def calculate_angle(self):
        b = self.dist_270
        c = self.dist_300
        a = math.sqrt(b**2 +c**2 - (2*b*c*math.cos(30)))    #cosine rule 

        C = math.asin((math.sin(30)/ a) * c)        #sine rule to find angles
        self.angle_to_wall = C





    def run_loop(self):
        msg = Twist()

        # stop if robot has crashed
        if self.dist_front <= 0.35:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            print(" stopped")
            return
        
        # forward velocity
        msg.linear.x = 0.2

        if (self.angle_to_wall < 90) and (self.dist_270 > self.target_distance):
            msg.angular.z = 0.0
        elif (self.angle_to_wall > 90) and (self.dist_270 < self.target_distance):
            msg.angular.z = 0.0
        elif (self.angle_to_wall > 90) and (self.dist_270 < self.target_distance):
            msg.angular.z = 1.0 #turn 
        elif (self.angle_to_wall > 90) and (self.dist_270 < self.target_distance):
            msg.angular.z = -1.0



        # angular velocity proportional to wall distance error
        # msg.angular.z = 0.4 * self.error
        msg.angular.z = self.pid(self.dist_right if not math.isinf(self.dist_right) else 2)
        print(f", zt {round(msg.angular.z, 3)}")
        
        # publish velocity command message
        self.vel_pub.publish(msg)

    # def run_loop(self):
    #     msg = Twist()

    #     # stop if robot has crashed
    #     if self.distance_to_obstacle <= 0.35:
    #         msg.linear.x = 0.0
    #         print(" stopped")
    #         return
        
    #     # go straight by default
    #     msg.linear.x = 0.2
    #     msg.angular.z = 0.0
        
    #     # if (self.dist_right < self.target_distance)\
    #     #         and (self.dist_right >= self.previous_distance):
    #     if (self.dist_right < self.target_distance):
    #         # too close and getting closer
    #         # turn right (towards wall)
    #         print(" turning right")
    #         msg.angular.z = -1.0
    #     # elif (self.dist_right > self.target_distance)\
    #     #         and (self.dist_right <= self.previous_distance):
    #     if (self.dist_right > self.target_distance):
    #         # too far and getting farther
    #         # turn left  (away from wall)
    #         print(" turning left")
    #         msg.angular.z = 1.0
        
    #     # publish velocity command message
    #     self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
