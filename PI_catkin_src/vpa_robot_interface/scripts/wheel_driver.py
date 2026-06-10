#!/usr/bin/python3

import rospy
import socket
from math import fabs, floor
import os
from dt_config.dt_hardware_settings import MotorDirection, HATv2
import yaml

import numpy as np

from vpa_robot_interface.msg import WheelsCmd,WheelsEncoder
from vpa_robot_interface.cfg import omegaConfig

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu

from pid_controller.wheel_speed_controller import WheelSpeedController

from dynamic_reconfigure.server import Server

class WheelDriver:

    LEFT_MOTOR_MIN_PWM  = 60        #: Minimum speed for left motor
    LEFT_MOTOR_MAX_PWM  = 255       #: Maximum speed for left motor
    RIGHT_MOTOR_MIN_PWM = 60        #: Minimum speed for right motor
    RIGHT_MOTOR_MAX_PWM = 255       #: Maximum speed for right motor
    SPEED_TOLERANCE     = 1.0e-2    #: Speed tolerance level

    def __init__(self) -> None:
        self.hat        = HATv2()
        self.leftMotor  = self.hat.get_motor(1, "left")
        self.rightMotor = self.hat.get_motor(2, "right")

        self.leftThrottle   = 0.0
        self.rightThrottle  = 0.0
        self._pwm_update()

    def set_wheels_throttle(self, left: float, right: float):
        """Sets speed of motors.

        Args:
           left (:obj:`float`): speed for the left wheel, should be between -1 and 1
           right (:obj:`float`): speed for the right wheel, should be between -1 and 1
           is_test_cmd (:obj:`bool`): whether this is a command issue by the hardware test

        """
        self.leftThrottle  = left
        self.rightThrottle = right
        self._pwm_update()

    def _pwm_value(self, v, min_pwm, max_pwm):
        """Transforms the requested speed into an int8 number.

        Args:
            v (:obj:`float`): requested speed, should be between -1 and 1.
            min_pwm (:obj:`int8`): minimum speed as int8
            max_pwm (:obj:`int8`): maximum speed as int8
        """
        pwm = 0
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (max_pwm - min_pwm) + min_pwm))
        return min(pwm, max_pwm)

    def _pwm_update(self):
        """Sends commands to the microcontroller.

        Updates the current PWM signals (left and right) according to the
        linear velocities of the motors. The requested speed gets
        tresholded.
        """
        vl = self.leftThrottle
        vr = self.rightThrottle

        pwml = self._pwm_value(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self._pwm_value(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)
        leftMotorMode   = 0
        rightMotorMode  = 0

        if fabs(vl) < self.SPEED_TOLERANCE:
            pwml = 0
        elif vl > 0:
            leftMotorMode = MotorDirection.FORWARD
        elif vl < 0:
            leftMotorMode = MotorDirection.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            pwmr = 0
        elif vr > 0:
            rightMotorMode = MotorDirection.FORWARD
        elif vr < 0:
            rightMotorMode = MotorDirection.BACKWARD

        self.leftMotor.set(leftMotorMode, pwml)
        self.rightMotor.set(rightMotorMode, pwmr)

    def __del__(self):
        """Destructor method.

        Releases the motors and deletes tho object.
        """
        self.leftMotor.set(MotorDirection.RELEASE)
        self.rightMotor.set(MotorDirection.RELEASE)
        del self.hat
    
class WheelDriverNode:

    def __init__(self) -> None:

        rospy.on_shutdown(self.shut_hook)
        self.veh_name       = socket.gethostname()
        self.direct_mode    = rospy.get_param('~direct_mode',False)

        self.driver = WheelDriver()
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.log_dir = os.path.join(script_dir, 'logs')
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        self.default_kp = (0.2, 0.2)   # or [0.3, 0.2]
        self.default_ki = (0.3, 0.3)


        self.wheel_spd_controller = WheelSpeedController(Kp=self.default_kp,Ki=self.default_ki)

        self.omega_left_ref     = 0
        self.omega_right_ref    = 0

        self.omega_left_sig     = 0
        self.omega_right_sig    = 0

        self.throttle_left      = 0
        self.throttle_right     = 0

        # Kinematics

        self._v_max     = 1         # max longitudinal speed m/s
        self._omega_max = 8         # max yaw rate rad/s
        self._baseline  = 0.1       # gap between wheels m
        self._radius    = 0.0318    # radius of wheels
        
        self.estop         = True
        self.local_estop   = True
        # Subscribers
        self.pub_wheel_debug    = rospy.Publisher('wheel_ref',WheelsCmd,queue_size=1)
        if not self.direct_mode:
            self.sub_wheel_enc = rospy.Subscriber("wheel_omega",WheelsEncoder,self.wheel_omega_cb,queue_size=1)
            self.sub_car_cmd   = rospy.Subscriber("cmd_vel", Twist, self.car_cmd_cb)
        else:
            self.sub_wheel_cmd = rospy.Subscriber("throttle",WheelsCmd,self.wheel_direct_cb,queue_size=1)
            
        self.sub_e_stop         = rospy.Subscriber("/global_brake", Bool, self.estop_cb, queue_size=1)
        self.sub_local_e_stop   = rospy.Subscriber("local_brake", Bool, self.estop_local_cb, queue_size=1)
        
        
        self.srv_wheel = Server(omegaConfig, self.dynamic_reconfigure_callback)
        self.srv_wheel.update_configuration({
            'kp_left': self.default_kp[0],
            'ki_left': self.default_ki[0],
            'kp_right': self.default_kp[1],
            'ki_right': self.default_ki[1]
        })

        pid_setting_path = os.path.join(self.log_dir, 'wheel_pid.yaml')

        if os.path.exists(pid_setting_path):

            with open(pid_setting_path, 'r') as f:
                try:
                    pid_params = yaml.safe_load(f)
                    kp_left = pid_params.get('left_kp', self.default_kp[0])
                    ki_left = pid_params.get('left_ki', self.default_ki[0])
                    kp_right = pid_params.get('right_kp', self.default_kp[1])
                    ki_right = pid_params.get('right_ki', self.default_ki[1])
                    self.wheel_spd_controller.update_gains(kp_left, ki_left, kp_right, ki_right)
                    rospy.loginfo(f"{self.veh_name}: Loaded PID params from {pid_setting_path}")
                except json.JSONDecodeError as e:
                    rospy.logwarn(f"{self.veh_name}: Failed to parse PID settings file: {e}")
        else:
            rospy.loginfo(f"{self.veh_name}: No existing PID settings file found at {pid_setting_path}, using default gains.")

        rospy.loginfo("%s: wheel drivers ready",self.veh_name)

    def signal_shut(self,msg:Bool):
        if msg.data:
            rospy.signal_shutdown('wheel driver node shutdown')

    def car_cmd_cb(self,msg_car_cmd:Twist) -> None:
        msg_car_cmd.linear.x    = max(min(msg_car_cmd.linear.x,self._v_max),-self._v_max)
        msg_car_cmd.angular.z   = max(min(msg_car_cmd.angular.z,self._omega_max),-self._omega_max)
        self.yaw_setpoint = msg_car_cmd.angular.z  # Negate the yaw setpoint
        self.omega_right_ref    = 0
        self.omega_left_ref     = 0
        if not self.estop:
            if msg_car_cmd.linear.x != 0:
                self.omega_right_ref    = ((msg_car_cmd.linear.x + 0.5 * msg_car_cmd.angular.z * self._baseline) / self._radius) 
                self.omega_left_ref     = ((msg_car_cmd.linear.x - 0.5 * msg_car_cmd.angular.z * self._baseline) / self._radius) 
                
        #print('ref',self.omega_left_ref,self.omega_right_ref)
        msg_wheel_cmd = WheelsCmd()
        msg_wheel_cmd.vel_left          = self.omega_left_ref
        msg_wheel_cmd.vel_right         = self.omega_right_ref
        msg_wheel_cmd.throttle_left     = self.throttle_left
        msg_wheel_cmd.throttle_right    = self.throttle_right
        self.pub_wheel_debug.publish(msg_wheel_cmd)

    def estop_cb(self,msg:Bool) -> None:
        if self.estop != msg.data:
            rospy.loginfo('%s: global brake: %s',self.veh_name,str(msg.data))
        self.estop = msg.data

    def estop_local_cb(self,msg:Bool) -> None:
        if self.local_estop != msg.data:
            rospy.loginfo('%s: local brake: %s',self.veh_name,str(msg.data))
        self.local_estop = msg.data        
    
    def shut_hook(self) -> None:
        self.estop = True
        self.driver.set_wheels_throttle(left=0,right=0)
        self.driver = None
        rospy.loginfo("%s: Wheel driver shutdown",self.veh_name)

    def wheel_direct_cb(self,msg:WheelsCmd) -> None:
        
        self.throttle_left  = msg.throttle_left
        self.throttle_right = msg.throttle_right
        
        if not self.estop and not self.local_estop:
            self.driver.set_wheels_throttle(left=self.throttle_left,right=self.throttle_right)
        else:
            self.driver.set_wheels_throttle(left=0,right=0)

    
    def wheel_omega_cb(self,msg:WheelsEncoder) -> None:

        self.omega_left_sig     = msg.omega_left
        self.omega_right_sig    = msg.omega_right

        omega = np.array([self.omega_left_sig, self.omega_right_sig])
        omega_ref = np.array([self.omega_left_ref, self.omega_right_ref])

        
        u = self.wheel_spd_controller.compute(omega, omega_ref)
        
        if omega_ref[0] == 0:
            u[0] = 0
        
        if omega_ref[1] == 0:
            u[1] = 0


        throttle_left  = u[0]
        throttle_right = u[1]

        bound = 0.3
        # Apply throttle limits
        if not throttle_left == 0:

            if throttle_left-self.throttle_left > bound:
                self.throttle_left = self.throttle_left + bound
            else:
                self.throttle_left = throttle_left

        else:
            self.throttle_left = 0

        if not throttle_right == 0:
            
            if throttle_right-self.throttle_right > bound:
                self.throttle_right = self.throttle_right + bound
            else:
                self.throttle_right = throttle_right
        else:
            self.throttle_right = 0


        if not self.estop and not self.local_estop:
            self.driver.set_wheels_throttle(left=self.throttle_left, right=self.throttle_right)
        else:
            self.driver.set_wheels_throttle(left=0, right=0)


    def dynamic_reconfigure_callback(self, config, level):
        # Debug: see exactly what’s coming in
        # print("cfg type:", type(config), "keys:", list(config.keys()))

        # Always cast to native Python floats
        kp_l = float(config.kp_left)
        ki_l = float(config.ki_left)
        kp_r = float(config.kp_right)
        ki_r = float(config.ki_right)

        self.wheel_spd_controller.update_gains(kp_l, ki_l, kp_r, ki_r)
        return config  # MUST return config unchanged
        

if __name__ == '__main__':

    try:
        rospy.init_node("wheel_driver")
        N = WheelDriverNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard Shutdown')
