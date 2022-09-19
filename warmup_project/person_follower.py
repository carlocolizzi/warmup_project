import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist # Neato control messages
from rclpy.qos import qos_profile_sensor_data

import time
import math
import numpy as np

class PersonFollower(Node):

    def __init__(self):
        # run superclass constructor
        super().__init__("laser_scan")

        self.create_subscription(LaserScan, 'scan', self.process_scan, qos_profile=qos_profile_sensor_data)   # lidar sub
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)      # velocity pub
        
    def process_scan(self, msg):
            # update state members from lidar subscription data
            for index in range(0,len(msg.ranges)):
                if math.isinf(msg.ranges[index]):
                    msg.ranges[index] = 0
            dist_front = msg.ranges

            nonzero_indices = np.flatnonzero(dist_front)
            for index in range(0,len(nonzero_indices)):

                remapped_indices = remap(nonzero_indices[index])
            self.heading = np.mean(remapped_indices)
            #self.heading = remap(self.heading)

            self.act()

    def act(self):
        msg = Twist()

        if self.heading < 0:
            msg.angular.z = -1.0
        elif self.heading > 0:
            msg.angular.z = 1.0
        else:
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            time.sleep(0.1)
            msg.linear.x = 1.0

        self.vel_pub.publish(msg)

def remap(number):
        if number < 180:
            result = number
        else:
            result = number -360
        return result

def main(args=None):
    rclpy.init(args=args)
    node = PersonFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
