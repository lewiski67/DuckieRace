#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import Imu 
from vpa_robot_interface.msg import WheelsEncoder
from math import pi, cos, sin, atan2, sqrt
import socket
import numpy as np
class PoseToTwistNode:
    def __init__(self):
        rospy.init_node('pose_to_twist_node')
        self.robot_name = socket.gethostname()
        # Params
        self.vf_ref      = rospy.get_param('~vf_ref', 0.30)
        self.Kw          = rospy.get_param('~Kw', 1.5)
        self.stop_radius = rospy.get_param('~stop_radius', 0.05)
        self.w_max       = rospy.get_param('~w_max', 3.0)
        self.ticks_per_rev = rospy.get_param('~ticks_per_rev', 135)
        self.wheel_radius  = rospy.get_param('~wheel_radius', 0.0318)
        self.wheel_base    = rospy.get_param('~wheel_base', 0.10)

        # State
        self.x = 0.0; self.y = 0.0; self.theta = 0.0
        self.theta_imu = 0.0
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.target_pose = Pose2D()

        # Derived
        # self.tick_to_meter = 2 * pi * self.wheel_radius / self.ticks_per_rev

        # IO
        rospy.Subscriber("target_pose", Pose2D, self.target_callback, queue_size=1)
        rospy.Subscriber("wheel_omega", WheelsEncoder, self.encoder_cb, queue_size=1)
        self.imu_gyro_z_init = []
        rospy.Subscriber("imu", Imu, self.imu_cb, queue_size=1)
        self.cmd_pub  = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pose_pub = rospy.Publisher("dead_reckoned_pose", Pose2D, queue_size=1)

        rospy.loginfo("%s: PoseToTwist initialized", self.robot_name)

    def target_callback(self, msg: Pose2D):
        self.target_pose = msg
        rospy.loginfo(f"{self.robot_name}: New target pose: x={msg.x}, y={msg.y}, theta={msg.theta}")

    def imu_cb(self, msg: Imu):
        w_z = msg.angular_velocity.z
        if len(self.imu_gyro_z_init) < 50:
            self.imu_gyro_z_init.append(w_z)
            if len(self.imu_gyro_z_init) == 50:
                self.imu_gyro_z_bias = np.mean(self.imu_gyro_z_init)
                rospy.loginfo(f"{self.robot_name}: IMU gyro z bias initialized: {self.imu_gyro_z_bias}")
            return
        w_z -= self.imu_gyro_z_bias
        dt = 1.0 / 20.0  # assuming imu at 20 Hz
        self.theta_imu -= w_z * dt
        

    def encoder_cb(self, msg: WheelsEncoder):
        curr_left_ticks  = msg.left_ticks
        curr_right_ticks = msg.right_ticks

        if self.last_left_ticks is None:
            self.last_left_ticks = curr_left_ticks
            self.last_right_ticks = curr_right_ticks
            return

        delta_l = (curr_left_ticks  - self.last_left_ticks) 
        delta_r = (curr_right_ticks - self.last_right_ticks)
        self.last_left_ticks  = curr_left_ticks
        self.last_right_ticks = curr_right_ticks

        d_l = 2 * np.pi * self.wheel_radius * (delta_l / self.ticks_per_rev)
        d_r = 2 * np.pi * self.wheel_radius * (delta_r / self.ticks_per_rev)

        d_center = 0.5 * (d_l + d_r)
        # d_theta  = (d_r - d_l) / self.wheel_base

        # Update pose
        self.theta = self.theta_imu  # use imu heading
        self.x     += d_center * np.cos(self.theta)
        self.y     += d_center * np.sin(self.theta)

        # # normalize heading
        # self.theta = atan2(sin(self.theta), cos(self.theta))
        # if delta_l != 0 or delta_r != 0:
        #     print(f"x={self.x}, y={self.y}, theta={self.theta}")
        # publish pose
        pose_msg = Pose2D(x=self.x, y=self.y, theta=self.theta)
        self.pose_pub.publish(pose_msg)

        # # compute and publish cmd
        v_ref, w_ref, _ = self.compute_twist(self.x, self.y, self.theta,
                                             self.target_pose.x, self.target_pose.y,self.target_pose.theta)
        cmd = Twist()
        cmd.linear.x  = v_ref
        cmd.angular.z = w_ref
        
        self.cmd_pub.publish(cmd)

    def compute_twist(self, x_meas, y_meas, theta_meas, x_ref, y_ref, theta_ref):
        # ---- fixed params for smooth turning ----
        vf_ref      = 0.30          # max forward speed (m/s)
        v_min_move  = 0.08          # minimum to overcome deadband (only when facing forward)
        stop_enter  = 0.05          # stop radius (m)
        near_scale  = 0.20          # start tapering v within this range (m)
        K_ang       = 0.5           # small P on heading
        w_max       = 2.0           # absolute cap on yaw rate (rad/s)
        alpha_w     = 0.3           # 0..1; higher = snappier ω
        theta_goal  = theta_ref     # desired final heading (rad)
        theta_tol   = 5.0 * pi/180  # done when |theta_err| < 5°
        wheel_base  = 0.10          # DB19 wheelbase (m)
        a_max       = 0.5           # braking accel (m/s^2)

        # keep filter state
        if not hasattr(self, 'prev_w'):
            self.prev_w = 0.0

        # ---- geometry ----
        dx = x_ref - x_meas
        dy = y_ref - y_meas
        distance = sqrt(dx*dx + dy*dy)
        target_bearing = atan2(dy, dx)

        def wrap(a): return atan2(sin(a), cos(a))
        angle_err = wrap(target_bearing - theta_meas)

        # ---- stop if at goal (then softly align heading) ----
        if distance < stop_enter:
            theta_err = wrap(theta_goal - theta_meas)
            if abs(theta_err) <= theta_tol:
                v_cmd, w_des = 0.0, 0.0
            else:
                v_cmd, w_des = 0.0, K_ang * theta_err

            # filter & clamp ω with dynamic limit (prevents negative wheel speeds)
            w_cmd = (1.0 - alpha_w) * self.prev_w + alpha_w * w_des
            w_limit = min(w_max, 2.0 * max(v_cmd, 0.01) / wheel_base)
            w_cmd = max(-w_limit, min(w_limit, w_cmd))
            self.prev_w = w_cmd

        # ---- move with smooth curvature (pure pursuit style) ----
        # speed taper (don’t force v_min when close or facing away)
        c = cos(angle_err)
        taper = min(1.0, distance / max(near_scale, 1e-6))
        v_cmd = vf_ref * taper * max(0.0, c)
        if distance < 0.05 or c < 0.2:
            v_cmd = 0.0
        else:
            v_cmd = max(v_min_move, v_cmd)

        # braking profile to stop_enter
        v_cmd = min(v_cmd, sqrt(max(0.0, 2.0 * a_max * (distance - stop_enter))))

        # bounded curvature via lookahead
        Ld = min(max(distance, 0.20), 0.50)     # 20–50 cm lookahead
        kappa = 2.0 * sin(angle_err) / Ld
        w_des = v_cmd * kappa + K_ang * angle_err

        # filter & clamp ω; ensure wheels stay non-negative
        w_cmd = (1.0 - alpha_w) * self.prev_w + alpha_w * w_des
        w_limit = min(w_max, 2.0 * max(v_cmd, 0.01) / wheel_base)
        w_cmd = max(-w_limit, min(w_limit, w_cmd))
        self.prev_w = w_cmd

        # info line
        rospy.loginfo_throttle(0.5, f'v_ref: {v_cmd:.3f}, w_ref: {w_cmd:.3f}, x_meas: {x_meas}, y_meas:{y_meas},theta_meas:{theta_meas}')

        return v_cmd, w_cmd, distance


if __name__ == '__main__':
    try:
        PoseToTwistNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
