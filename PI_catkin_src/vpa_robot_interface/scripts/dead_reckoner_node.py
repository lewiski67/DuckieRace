#!/usr/bin/env python3

import math, rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import socket
from vpa_robot_interface.msg import WheelsEncoder

def wrap(a): return math.atan2(math.sin(a), math.cos(a))

class DeadReckonerNode:

    def __init__(self):
        rospy.init_node('dead_reckoner_node')
        self.robot_name = socket.gethostname()
        self.ticks_per_rev = 135
        self.wheel_radius  = 0.0318
        self.wheel_base    = 0.10
        
        self.tick_to_meter = 2 * math.pi * self.wheel_radius / self.ticks_per_rev

        self.start_pose = Pose2D()
        
        self.dd_in_operation = False

        rospy.Subscriber("reset_odometry", Bool, self.reset_odometry_cb, queue_size=1)
        self.start_pose = Pose2D(0.0, 0.0, 0.0)
        rospy.Subscriber("start_pose", Pose2D, self.start_pose_cb, queue_size=1)

        self.curr_left_ticks = None
        self.curr_right_ticks = None

        rospy.Subscriber("wheel_omega", WheelsEncoder, self.encoder_cb, queue_size=1)

        self.imu_gyro_z_init = []
        rospy.Subscriber("imu", Imu, self.imu_cb, queue_size=1)
        self.pose_pub = rospy.Publisher("dead_reckoned_pose", Pose2D, queue_size=1)

        rospy.loginfo("%s: DeadReckonerNode initialized", self.robot_name)

        self.prev_ticks_left = None
        self.prev_ticks_right = None
        self.x = None
        self.y = None
        self.theta = None

    def reset_odometry_cb(self, msg: Bool):
        if self.dd_in_operation and msg.data:
            rospy.logwarn(f"{self.robot_name}: Cannot reset odometry while dead reckoning in operation")
            return
        if msg.data:
            self.dd_in_operation = True
            self.x = self.start_pose.x
            self.y = self.start_pose.y
            self.theta = self.start_pose.theta

            self.prev_ticks_left = self.curr_left_ticks
            self.prev_ticks_right = self.curr_right_ticks
            rospy.loginfo(f"{self.robot_name}: Dead reckoning started at x={self.x}, y={self.y}, theta={self.theta}")
            rospy.loginfo(f"{self.robot_name}: Encoder ticks initialized at left={self.prev_ticks_left}, right={self.prev_ticks_right}")
        else:
            self.dd_in_operation = False
            self.x = 0
            self.y = 0
            self.theta = 0
            self.prev_ticks_left = None
            self.prev_ticks_right = None
            rospy.loginfo(f"{self.robot_name}: Dead reckoning stopped and reset to zero")


    def start_pose_cb(self, msg: Pose2D):
        if self.dd_in_operation:
            return # not accept new start pose while in operation
        self.start_pose = msg
        rospy.loginfo(f"{self.robot_name}: [DEAD-RECKONING] New start pose: x={msg.x}, y={msg.y}, theta={msg.theta}")

    def imu_cb(self, msg: Imu):
        w_z = msg.angular_velocity.z
        if len(self.imu_gyro_z_init) < 50:
            self.imu_gyro_z_init.append(w_z)
            if len(self.imu_gyro_z_init) == 50:
                self.imu_gyro_z_bias = sum(self.imu_gyro_z_init) / len(self.imu_gyro_z_init)
                rospy.loginfo(f"[DEAD-RECKONING] {self.robot_name}: IMU gyro z bias initialized: {self.imu_gyro_z_bias}")
            return
        w_z -= self.imu_gyro_z_bias
        dt  = 0.05  # assuming imu at 20 Hz
        if self.theta is not None:
            self.theta -= w_z * dt
            self.theta = wrap(self.theta)

    def encoder_cb(self, msg: WheelsEncoder):
        
        self.curr_left_ticks  = msg.left_ticks
        self.curr_right_ticks = msg.right_ticks

        if not self.dd_in_operation:
            return
        
        if self.prev_ticks_left is None or self.prev_ticks_right is None:
            self.prev_ticks_left = self.curr_left_ticks
            self.prev_ticks_right = self.curr_right_ticks
            return
        
        delta_l = (self.curr_left_ticks  - self.prev_ticks_left)
        delta_r = (self.curr_right_ticks - self.prev_ticks_right)
        self.prev_ticks_left  = self.curr_left_ticks
        self.prev_ticks_right = self.curr_right_ticks

        d_l = delta_l * self.tick_to_meter
        d_r = delta_r * self.tick_to_meter
        d_center = 0.5 * (d_l + d_r)
        if self.theta is None:
            return
        delta_x = d_center * math.cos(self.theta)
        delta_y = d_center * math.sin(self.theta)
        if abs(delta_x) > 0.5 or abs(delta_y) > 0.5:
            rospy.logwarn(f"{self.robot_name}: Large jump in position detected: delta_x={delta_x}, delta_y={delta_y}. With encoder ticks delta_l={delta_l}, delta_r={delta_r}. Current enc ticks left={self.curr_left_ticks}, right={self.curr_right_ticks}, previous enc ticks left={self.prev_ticks_left}, right={self.prev_ticks_right}. Checking this update.")

        self.x += delta_x
        self.y += delta_y
        self.theta = wrap(self.theta)  # already updated in imu_cb


        pose_msg = Pose2D()
        pose_msg.x = self.x
        pose_msg.y = self.y
        pose_msg.theta = self.theta
        self.pose_pub.publish(pose_msg)
    
if __name__ == "__main__":
    node = DeadReckonerNode()
    rospy.spin()