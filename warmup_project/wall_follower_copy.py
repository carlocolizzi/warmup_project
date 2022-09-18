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
        
        # line follower params
        self.target_distance = 1.0 # (m)   dist to follow wall at
        self.look_ahead_deg  = 30  # (deg) angle of look-ahead vector

        # state members
        self.dist_front = None     # (m) current distance dead ahead
        self.look_right = None     # (m) current look-right dist
        self.look_ahead = None     # (m) current look-ahead dist
        self.dist_to_wall = None   # (m) current calculated dist to wall

        # PID controller
        self.pid = PID(2.0, 0.5, 3, setpoint=1.0)
    
    def process_scan(self, msg):
        # update state members from lidar subscription data
        self.dist_front = msg.ranges[0]
        self.look_right = msg.ranges[270]
        self.look_ahead = msg.ranges[290]

        # TODO: handle inf edge cases

        # calculate and update distance to wall
        self.update_dist_to_wall()
        
        self.run_loop()

        # debug
        print(f"right {round(self.look_right, 2)}, "\
             +f"front {round(self.dist_front, 2)}, "\
             +f"ahead {round(self.look_ahead, 2)}, "\
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
        msg.linear.x = 0.3

        # angular velocity proportional to wall distance error
        # msg.angular.z = 0.4 * self.error
        msg.angular.z = self.pid(self.dist_to_wall)
        msg.angular.z = max(msg.angular.z, -2)
        print(f", zt {round(msg.angular.z, 3)}")
        
        # publish velocity command message
        self.vel_pub.publish(msg)
    
    def update_dist_to_wall(self):
        """
        Find distance from neato to wall (on right side)

        Args:
            lr (float): distance from 270deg laser
            la (float): distance from look-ahead laser
            la_deg (float): angle between lookright and lookahead vectors
        """
        # convert angle to rad
        theta = math.radians(self.look_ahead_deg)
        # calculate area of triangle formed by look-right vector, look-ahead vector, and wall
        area = abs(0.5 * self.look_right * self.look_ahead * math.sin(theta))
        # calculate length of wall between look-right and look-ahead points
        base = math.sqrt(self.look_right**2 + self.look_ahead**2 - 2*self.look_right*self.look_ahead*math.cos(theta))
        # calculate distance to wall by finding the triangle's height
        self.dist_to_wall = 2 * area / base

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
