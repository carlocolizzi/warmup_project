## Carlo Colizzi, Ben Grant
# 09.20.2022

# import ROS2 libraries
from turtle import heading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose2D, Vector3Stamped # Neato control messages
from rclpy.qos import qos_profile_sensor_data

# import python libraries
import time
import math
import numpy as np

# import installed libraries
from simple_pid import PID
from vector_2d import Vector, VectorPolar

class FiniteStateController(Node):

    def __init__(self):
        # run superclass constructor
        super().__init__("laser_scan")

        # create subscribers and publishers
        self.create_subscription(LaserScan, 'scan', self.detect_object,\
                                 qos_profile=qos_profile_sensor_data)      # lidar sub
        self.create_subscription(Odometry, 'odom', self.process_odom, 10)  # odometry sub
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)         # velocity pub
        # self.debug_pub = self.create_publisher(PointStamped, 'odom', 10)   # debug point pub

        # run loop timer
        timer_period = 0.1
        self.action_timer = self.create_timer(timer_period, self.decide_finite_state)

        # finite state machine parameters
        self.approach_threshold = 1.1 # (m) dist to switch from approaching to orbiting object
        # self.steer_pid  = PID(Kp=0.05, Ki=0, Kd=500, setpoint=0)    # PID controller for steering
        self.steer_pid  = PID(Kp=0.1, Ki=0, Kd=0.01, setpoint=0)    # PID controller for steering

        self.linvel_pid = PID(Kp=0.5,  Ki=0, Kd=0,   setpoint=1.0)  # PID controller for linear velocity

        # declare state members
        self.state = 0 # 0 for drive to target, 1 for orbit, 2 for do nothing
        self.scan           = []                    # laser scan array
        self.target_pos     = Vector(0, 10)         # (m) target pose
        self.object_pos     = Vector(0, 0)          # (m) detected object pose
        self.pose           = Pose2D(x=0.0, y=0.0)  # (m, deg) current pose
        self.dist_to_object = 100                   # (m) current dist to target point   
        self.dist_to_wall = 1.0           
        self.wall_target = 0.8

    # getter for Neato pose position
    def get_position(self):
        return Vector(self.pose.x, self.pose.y)

    def process_odom(self, msg):
        """
        Runs every time a new /odom msg rolls in. Updates pose state member.

        Args:
            msg (ROS Odometry): ROS /odom msg
        """
        # update pose state member from /odom pose msg
        position = msg.pose.pose.position
        self.pose.x = position.x; self.pose.y = position.y

        heading_deg = quat_to_yaw_deg(msg.pose.pose.orientation)
        self.pose.theta = heading_deg

    def detect_object(self, msg):
        """
        Detect object via LIDAR and write its "pose" (with theta=0)
        in global coordinates to a state member

        Args:
            msg (ROS LaserScan): Neato laser scan msg
        """
        # for orbit:
        self.dist_to_wall = min(msg.ranges[180:])

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
        
        def get_cartestian(polar_vec):
            # this is not correct but it works so fuck off
            return Vector( polar_vec.module*math.sin(polar_vec.angle), \
                           polar_vec.module*math.cos(polar_vec.angle))

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

        ## TODO: handle no object detected

        # get local polar coords of detected object (average of detected points)
        obj_lp_rad =  math.radians(convert_to_heading_deg(np.mean(obstacle_indicies)))
        obj_lp_range = np.mean(np.array(msg.ranges)[obstacle_indicies])

        # calculate global coords of object and write to obstacle_pos state member
        obj_lp_spin = VectorPolar(obj_lp_range, obj_lp_rad - math.radians(self.pose.theta))
        self.object_pos = self.get_position() + get_cartestian(obj_lp_spin)

        # calculate distance of neato to object and update state member
        self.dist_to_object = abs(-self.get_position() + self.object_pos)
    
    def decide_finite_state(self):
        # decide finite state based on threshold dist to target point
        if np.isnan(self.dist_to_object):
            self.state = 2
            # do nothing if no object detected
            drive_msg = Twist()
            self.vel_pub.publish(drive_msg)
        elif self.dist_to_object < self.approach_threshold:
            # on state switch
            if not self.state == 1:
                self.steer_pid = PID(0.5, 0.5, 5, setpoint=self.wall_target)

                # turn 90 deg left
                msg = Twist()
                msg.angular.z = 1.0; msg.linear.x = 0.0
                self.vel_pub.publish(msg)
                time.sleep(1.5)
                msg.linear.x = 0.0; msg.angular.z = 0.0
                self.vel_pub.publish(msg)
                time.sleep(0.5)

                self.state = 1

            self.orbit_object()
        else:
            # on state switch
            if not self.state == 0:
                self.steer_pid = PID(Kp=0.05, Ki=0, Kd=500, setpoint=0)
                self.state = 0
            # above threshold, follow object
            self.target_pos = self.object_pos
            self.drive_to_target()

    def orbit_object(self):
        """
        Drive circles around detected object.
        (Copied from wall follower)
        """
        msg = Twist()
        
        # forward velocity
        msg.linear.x = 0.2

        # angular velocity proportional to wall distance error
        msg.angular.z = self.steer_pid(self.dist_to_wall)
        
        # keep neato from spinning out of control if the PID screws up!
        if  np.isnan(msg.angular.z) or np.isinf(msg.angular.z):
            msg.angular.z = 0.0

        print(f"wall {self.dist_to_wall}, zt {round(msg.angular.z, 3)}")
        
        # publish velocity command message
        self.vel_pub.publish(msg)

    def drive_to_target(self):
        """
        Drive Neato to target point until within a target distance
        """
        # # unit vector with neato's heading
        # neato_heading = VectorPolar(1, math.radians(self.pose.theta)).to_cartesian()

        # displacement vector from neato to target
        neato_to_target = -self.get_position() + self.target_pos

        # desired heading will be the signed angle between these vectors
        # desired_theta = math.degrees(signed_angle(neato_heading, neato_to_target)) - 90
        desired_theta = math.degrees(signed_angle(Vector(0.0,1.0), neato_to_target))
        if -270 <= desired_theta < -180:
            desired_theta = desired_theta + 360

        drive_msg = Twist()

        # use PID control to steer neato towards desired heading
        self.steer_pid.setpoint = desired_theta
        # drive_msg.angular.z = self.steer_pid(0)
        drive_msg.angular.z = self.steer_pid(self.pose.theta)


        # use PID control to slow down approach to target
        drive_msg.linear.x =  -self.linvel_pid(self.dist_to_object)
        drive_msg.linear.x = min(drive_msg.linear.x, 1.0)

        self.vel_pub.publish(drive_msg)

        self.print_debug()
        print(f" | dth {round(self.steer_pid.setpoint, 2)}")
    
    def print_debug(self):
        print("pos | "\
             +f"x: {round(self.pose.x, 2)} | "\
             +f"y: {round(self.pose.y, 2)} | "\
             +f"theta_deg: {round(self.pose.theta, 2)} | "\
             + "obj | "\
             +f"x: {round(self.object_pos.x, 2)} | "\
             +f"y: {round(self.object_pos.y, 2)} | "\
             +f"d2o {round(self.dist_to_object, 2)}",\
                end="")


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

def signed_angle(a, b):
    """
    Return signed angle fron first to second vector

    Args:
        a (Vector): first vector
        b (Vector): second vector
    """
    # return math.atan2(a.x*b.y - a.y*b.x, a.x*b.x + a.y*b.y)
    return math.atan2(b.y,b.x) - math.atan2(a.y,a.x)

def quat_to_yaw_deg(q):
    """
    Extract just the yaw component (rotation about the z axis)
    from an [x,y,z,w] quaternion

    Args:
        q (struct): quaternion

    Returns:
        float: yaw in degrees
    """
    yaw_rad = math.atan2(2.0 * (q.w * q.z + q.x * q.y), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)

    return math.degrees(yaw_rad)

def remap(number):
        if number <= 180:
            result = number
        else:
            result = number - 360
        return result

def main(args=None):
    rclpy.init(args=args)
    node = FiniteStateController()
    rclpy.spin(node)
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()