from cmath import isinf, isnan
from turtle import distance
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3 # Neato control messages
from rclpy.qos import qos_profile_sensor_data

import numpy as np
import math
from simple_pid import PID
from statistics import mean

class Subscriber(Node):

    def __init__(self):
        # run superclass constructor
        super().__init__("laser_scan")

        self.create_subscription(LaserScan, 'scan', self.process_scan,\
                                 qos_profile=qos_profile_sensor_data)   # lidar sub
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)      # velocity pub
        
        # algorithm params
        self.target_distance = 1.0 # (m)   dist to follow wall at
        self.pid = PID(3, 0.5, 5, setpoint=self.target_distance)

        # state members
        self.dist_front = None     # (m) current distance dead ahead
        self.dist_to_wall = None   # (m) current calculated dist to wall
        
    
    def process_scan(self, msg):
        # update state members from lidar subscription data
        self.dist_front = msg.ranges[0]

        # TODO: handle inf edge cases

        # calculate and update distance to wall
        # self.update_dist_to_wall()
        self.dist_to_wall = min(msg.ranges[180:])
        
        self.run_loop()

        # debug
        print(f"front {round(self.dist_front, 2)}, "\
             +f"wall { round(self.dist_to_wall, 2)}", end="")

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

        # angular velocity proportional to wall distance error
        msg.angular.z = self.pid(self.dist_to_wall)
        
        # keep neato from spinning out of control if the PID screws up!
        if  isnan(msg.angular.z) or isinf(msg.angular.z):
            msg.angular.z = 0.0

        print(f", zt {round(msg.angular.z, 3)}")
        
        # publish velocity command message
        self.vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()