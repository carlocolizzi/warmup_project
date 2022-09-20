from pdb import post_mortem
# from pymunk.vec2d import Vec2d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose2D, PointStamped, Point
from rclpy.qos import qos_profile_sensor_data

import time
import math
import numpy as np

from vector_2d import Vector, VectorPolar

class ObstacleAvoider(Node):

    def __init__(self):
        # run superclass constructor
        super().__init__("laser_scan")

        # create subscribers and publishers
        self.create_subscription(LaserScan, 'scan', self.process_scan,\
                                 qos_profile=qos_profile_sensor_data)      # lidar sub
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)  # odometry sub
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)         # velocity pub
        self.debug_pub = self.create_publisher(PointStamped, 'odom', 10)     # debug poses pub

        # declare state members
        self.target_pos    = Vector(0, 10)         # (m) target pose
        self.obstacle_pos  = Vector(0, 0)          # (m) detected obstacle pose
        self.pose          = Pose2D(x=0.0, y=0.0) # (m, deg) current pose

    # getter for Neato pose position
    def get_position(self):
        return Vector(self.pose.x, self.pose.y)

    def process_odom(self, msg):
        """
        Runs every time a new /odom msg rolls in. Updates pose state member and prints debug info.

        Args:
            msg (ROS Odometry): ROS /odom msg
        """
        # update pose state member from /odom pose msg
        position = msg.pose.pose.position
        heading_deg = quat_to_yaw_deg(msg.pose.pose.orientation)
        self.pose.x = position.x; self.pose.y = position.y
        self.pose.theta = heading_deg

        # debug
        print_same_line("pos | "\
             +f"x: {round(self.pose.x, 2)} | "\
             +f"y: {round(self.pose.y, 2)} | "\
             +f"theta_deg: {round(self.pose.theta, 2)} | ")

    def process_scan(self, msg):
        """
        Detect object via LIDAR and write its "pose" (with theta=0)
        in global coordinates to a state member

        Args:
            msg (ROS LaserScan): Neato laser scan msg
        """
        def remap_scan(ranges):
            """
            Reorders a laser scan list to a form where element 0 is
            directly behind the neato with positive elements up to
            359 going around clockwise. Element 90 is left,
            element 180 is in front, and element 270 is right

            Args:
                ranges (list of floats): laser scan list

            Returns:
                list of floats: remapped scan list
            """
            # split list into halfs
            left_half  = ranges[0:179]
            right_half = ranges[180:]

            # reverse halfs and concatenate
            return left_half[::-1] + right_half[::-1]
        
        def convert_to_heading_deg(index):
            return index - 180

        # clean msg.ranges (make inf = 0)
        for degree, _range in enumerate(msg.ranges):
            if math.isinf(_range):
                msg.ranges[degree] = 0
        
        # remap msg.ranges
        msg.ranges = remap_scan(msg.ranges)

        # nonzero scan elements will belong to the detected obstacle
        obstacle_indicies = np.flatnonzero(msg.ranges)

        # remap the ranges list such that the list wrapping occurs
        # directly behind the neato instead of directly in front
        remapped_indices = []
        for index in range(0,len(obstacle_indicies)):
            remapped_indices.append(remap(obstacle_indicies[index]))
        print(remapped_indices)

        ## TODO: handle no object detected

        # local polar coords of detected object (average of detected points)
        obj_lp_deg =  convert_to_heading_deg(np.mean(obstacle_indicies))
        obj_lp_range = np.mean(np.array(msg.ranges)[obstacle_indicies])

        # convert local polar  coords to global coord and write to obj pose state member
        self.obstacle_pos.set_comp(0, -(self.get_position().x - math.cos(math.radians(obj_lp_deg))*obj_lp_range))
        self.obstacle_pos.set_comp(1, -(self.get_position().y + math.sin(math.radians(obj_lp_deg))*obj_lp_range))

        # run control loop
        self.run_loop()
        
    def run_loop(self):
        """
        Calculate and send velocity commands to the neato.
        Drive towards target pose and avoid detected objects.
        """
        # get (normalized) attraction vector that points from Neato's pose to goal pose
        att_vec = (self.target_pos - self.get_position())
        att_vec = att_vec*(1/abs(att_vec))          # normalize

        # get repulsion vector (normalized) that points from the obstacle towards the Neato
        rep_vec = self.obstacle_pos - self.get_position()
        rep_vec = rep_vec*(1/abs(rep_vec))          # normalize

        # calculate ideal force vector steers away from obstacle and towards target pos
        ideal_vec = att_vec + rep_vec
        ideal_vec = ideal_vec*(1/abs(ideal_vec))

        # calculate ideal heading from ideal force vector's angle to y-axis
        ideal_heading_deg = math.degrees(math.atan2(ideal_vec.y - 1, ideal_vec.x))
        print(f"id_heading {ideal_heading_deg}")

        # send velocity cmd to neato
        # self.drive_to_vector(angle)

        # debug publish
        msg_header = Header(stamp=self.get_clock().now().to_msg(), frame_id="odom")
        my_point_stamped = PointStamped(header=msg_header, point=Point(x=self.obstacle_pos.x, y=self.obstacle_pos.y, z=0.0))
        self.debug_pub.publish(my_point_stamped)


    def drive_to_vector(self, angle):
        msg = Twist()

        if angle < 0:
            msg.angular.z = -1.0
        elif angle > 0:
            msg.angular.z = 1.0
        else:
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            #time.sleep(0.1)
            msg.linear.x = 1.0

        self.vel_pub.publish(msg)


def quat_to_yaw_deg(q):
    """
    Extract just the yaw component (rotation about the z axis)
    from an [x,y,z,w] quaternion

    Args:
        q (struct): quaternion

    Returns:
        float: yaw in degrees
    """
    yaw_rad = math.atan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z)

    return math.degrees(yaw_rad)

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
