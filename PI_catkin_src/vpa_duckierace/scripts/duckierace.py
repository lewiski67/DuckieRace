#!/usr/bin/env python3

# import rospy
# from sensor_msgs.msg import Image, Joy, Range
# from std_msgs.msg import Bool, Float32, Int32
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import time
# import socket

# from toolbox.lane_detector import LaneDetector
# from toolbox.color_detector import ColorDetector
# from vpa_robot_interface.msg import WheelsEncoder

# class DuckieRaceNode:
#     def __init__(self):
#         rospy.init_node('duckierace_node')
#         self.robot_name = socket.gethostname()
#         self.bridge = CvBridge()

#         # ── 跟线相关 ──────────────────────────────────────────
#         self.lane_detector  = LaneDetector()        # 默认：黄白线边界
#         self.color_detector = ColorDetector()       # B键：直接跟黄线

#         self.image_half_width = 160                 # 图像半宽，默认320/2
#         self.last_center_x    = None
#         self.last_error       = 0.0
#         self.last_time        = rospy.Time.now()
#         self.lost_detect      = False

#         # PD参数（跟黄线用，黄白线模式用lane_detector内部逻辑）
#         self.kp = rospy.get_param('~kp', 2.0)
#         self.kd = rospy.get_param('~kd', 1.0)

#         # B键：是否切换到跟黄线模式
#         self.yellow_mode = False
#         self.yellow_last_seen = rospy.Time.now()

#         # ── 速度相关 ──────────────────────────────────────────
#         self.speed      = rospy.get_param('~init_speed', 0.3)
#         self.speed_step = 0.02
#         self.max_speed  = 0.35
#         self.min_speed  = 0.2

#         # ── ToF 前向防碰撞 ────────────────────────────────────
#         self.z_min       = 0
#         self.z_max       = 1.5
#         self.tof_range   = self.z_max + 0.1         # 安全默认值
#         self.tof_status  = 0
#         self.tof_warn    = False
#         rospy.Subscriber('front_range_status', Int32, self.tof_status_callback, queue_size=1)
#         rospy.Subscriber('front_range', Range, self.tof_callback, queue_size=1)

#         # ── 刹车 ──────────────────────────────────────────────
#         self.local_brake     = True
#         self.global_brake    = True
#         self.last_brake_sent = False
#         self.in_fuel_zone    = False
#         self.charging        = False
#         rospy.Subscriber('local_brake', Bool, self.local_brake_callback, queue_size=1)
#         rospy.Subscriber('/global_brake', Bool, self.global_brake_callback, queue_size=1)
#         rospy.Subscriber('in_fuel_zone', Bool, self.fuel_callback, queue_size=1)

#         # ── 手柄 ──────────────────────────────────────────────
#         self.last_joy_time = 0
#         rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

#         # ── 发布者 ────────────────────────────────────────────
#         self.cmd_pub   = rospy.Publisher('cmd_vel', Twist, queue_size=1)
#         self.brake_pub = rospy.Publisher(f'/{self.robot_name}/local_brake', Bool, queue_size=1)

#         # ── 摄像头 ────────────────────────────────────────────
#         rospy.Subscriber('robot_cam/image_raw', Image, self.image_callback, queue_size=1)

#         rospy.loginfo(f"[DuckieRace] Node initialized. Robot: {self.robot_name}, Init speed: {self.speed:.2f}")

#     # ── ToF 回调 ──────────────────────────────────────────────
#     def tof_status_callback(self, msg):
#         self.tof_status = msg.data

#     def tof_callback(self, msg):
#         if self.tof_status == 9:
#             self.tof_range = np.clip(msg.range - 0.04, self.z_min, self.z_max)
#         else:
#             self.tof_range = self.z_max + 0.1  # 无效读数视为安全
#         # rospy.loginfo(f"[ToF] status={self.tof_status} range_raw={msg.range:.3f} tof_range={self.tof_range:.3f}")
#     # def tof_callback(self, msg):
#     #     if self.tof_status == 9:
#     #         if msg.range < 0.04:  # 低于最小量程，无效读数
#     #             self.tof_range = self.z_max + 0.1  # 视为安全
#     #         else:
#     #             self.tof_range = np.clip(msg.range - 0.04, self.z_min, self.z_max)
#     #     else:
#     #         self.tof_range = self.z_max + 0.1

#     # ── 刹车回调 ──────────────────────────────────────────────
#     def local_brake_callback(self, msg):
#         self.local_brake = msg.data

#     def global_brake_callback(self, msg):
#         self.global_brake = msg.data

#     def fuel_callback(self, msg):
#         self.in_fuel_zone = msg.data

#     # ── 手柄回调 ──────────────────────────────────────────────
#     def joy_callback(self, msg):
#         now = time.time()
#         if now - self.last_joy_time < 0.6:
#             return
#         self.last_joy_time = now

#         # L1 加速
#         if msg.buttons[4]:
#             self.speed = min(self.speed + self.speed_step, self.max_speed)
#             rospy.loginfo(f"[DuckieRace] Speed increased to: {self.speed:.2f}")

#         # R1 减速
#         if msg.buttons[5]:
#             self.speed = max(self.speed - self.speed_step, self.min_speed)
#             rospy.loginfo(f"[DuckieRace] Speed decreased to: {self.speed:.2f}")

#         # X键 刹车控制
#         if msg.buttons[2] and not self.last_brake_sent:
#             if not self.in_fuel_zone:
#                 self.brake_pub.publish(Bool(data=False))
#                 self.last_brake_sent = True
#                 rospy.loginfo(f"[DuckieRace] Brake released")
#             else:
#                 if msg.buttons[1]:  # 燃料区内需同时按B
#                     self.brake_pub.publish(Bool(data=False))
#                     self.last_brake_sent = True
#                     rospy.loginfo(f"[DuckieRace] Brake released in fuel zone")
#         elif msg.buttons[2] and self.last_brake_sent:
#             if self.in_fuel_zone:
#                 self.brake_pub.publish(Bool(data=True))
#                 self.last_brake_sent = False
#                 rospy.loginfo(f"[DuckieRace] Brake engaged in fuel zone")

#         # Y键 充电
#         self.charging = bool(msg.buttons[3]) and self.in_fuel_zone

#         # B键 切换黄线模式
#         self.yellow_mode = bool(msg.buttons[1])

#     # ── 图像回调：核心跟线逻辑 ────────────────────────────────
#     def image_callback(self, msg):
#         # rospy.loginfo_throttle(2.0, f"[Mode] {'黄线模式' if self.yellow_mode else '黄白线边界模式'}")
#         # ToF 紧急停车
#         if self.tof_range < 0.1:
#             if not self.tof_warn:
#                 rospy.logwarn("[DuckieRace] Obstacle too close! Stopping.")
#                 self.tof_warn = True
#             self.publish_cmd(0.0, 0.0)
#             return
#         else:
#             if self.tof_warn:
#                 rospy.loginfo("[DuckieRace] Obstacle cleared.")
#                 self.tof_warn = False

        
#         # 刹车检查
#         if self.local_brake or self.global_brake:
#             self.publish_cmd(0.0, 0.0)
#             return

#         # 图像转换
#         try:
#             bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#         except Exception as e:
#             rospy.logerr(f"[DuckieRace] Image conversion error: {e}")
#             return
#         auto_yellow = self.tof_range < 0.70  # 30cm内自动切换
#         rospy.loginfo_throttle(0.5, f"[ToF] tof_range={self.tof_range:.3f} auto_yellow={auto_yellow} yellow_mode={self.yellow_mode}")

#         if auto_yellow or self.yellow_mode:
#             self._follow_yellow_line(bgr)
#         else:
#             self._follow_lane_boundaries(bgr)

#         # if self.yellow_mode:
#         #     # ── B键按下：直接跟黄线（ColorDetector方式）──────
#         #     self._follow_yellow_line(bgr)
#         # else:
#         #     # ── 默认：黄白线边界方式（LaneDetector方式）──────
#         #     self._follow_lane_boundaries(bgr)

#     def _follow_lane_boundaries(self, bgr):
#         """原程序跟线方式：检测黄线左边界+白线右边界，走中间"""
#         result = self.lane_detector.image_process(bgr)

#         if result is None or len(result) < 2:
#             rospy.logwarn_once("[DuckieRace] Lane boundaries not detected.")
#             self.publish_cmd(0.0, 0.0)
#             return

#         left_boundary  = result[0]
#         right_boundary = result[1]
#         lane_center_x  = (left_boundary + right_boundary) / 2

#         # 平滑滤波
#         if self.last_center_x is None:
#             self.last_center_x = lane_center_x

#         if abs(self.last_center_x - lane_center_x) <= 80:
#             lane_center_x  = 0.8 * lane_center_x + 0.2 * self.last_center_x
#             self.last_center_x = lane_center_x

#             error     = lane_center_x - self.image_half_width
#             now       = rospy.Time.now()
#             dt        = (now - self.last_time).to_sec()
#             derror    = (error - self.last_error) / dt if dt > 0 else 0.0
#             angular_z = -(self.kp * error + self.kd * derror) / self.image_half_width
#             angular_z = np.clip(angular_z, -2.0, 2.0)

#             self.last_error = error
#             self.last_time  = now
#             self.publish_cmd(self.speed, angular_z)
#         else:
#             # 变化太大，保持原速直行
#             self.publish_cmd(self.speed, 0.0)

#     def _follow_yellow_line(self, bgr):
#         """B键模式：直接跟黄线中心（ColorDetector方式）"""
#         mask = self.color_detector.get_mask(bgr, 'yellow')
#         h, w = mask.shape
#         # y = int(h * 0.6)
#         # line_row = mask[y, :]
#         # indices  = np.where(line_row > 0)[0]
#         y_start = int(h * 0.6)
#         y_end   = int(h * 0.8)
#         roi     = mask[y_start:y_end, :]
#         indices = np.where(roi > 0)[1]  # 注意是[1]取列坐标

#         if len(indices) == 0:
#             if not self.lost_detect:
#                 rospy.logwarn("[DuckieRace] Yellow line not detected!")
#                 self.lost_detect = True
#             # 黄线丢失时保持上一帧速度直行
#             # self.publish_cmd(self.speed, 0.0)
#             # self.publish_cmd(self.speed * 0.5, 0.5)
#             if self.auto_yellow:
#                 self.publish_cmd(self.speed * 0.5, 1.5)  # 强制大角度左转
#             else:
#                 self.publish_cmd(self.speed * 0.5, 0.5)
#             return

#         self.lost_detect = False
#         avg_x     = int(np.mean(indices))
#         center_x  = w // 2
#         # error     = (avg_x - center_x) / center_x
#         offset = int(w * 0.2)  # 向左偏10%宽度
#         error  = (avg_x - (center_x - offset)) / center_x

#         now    = rospy.Time.now()
#         dt     = (now - self.last_time).to_sec()
#         derror = (error - self.last_error) / dt if dt > 0 else 0.0

#         angular_z = -self.kp * error - self.kd * derror
#         angular_z = np.clip(angular_z, -2.0, 2.0)

#         self.last_error = error
#         self.last_time  = now
#         self.publish_cmd(self.speed, angular_z)

#     def publish_cmd(self, linear, angular):
#         msg = Twist()
#         msg.linear.x  = linear
#         msg.angular.z = angular
#         self.cmd_pub.publish(msg)

#     def run(self):
#         rospy.spin()

# if __name__ == '__main__':
#     node = DuckieRaceNode()
#     node.run()


#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, Joy, Range
from std_msgs.msg import Bool, Float32, Int32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import socket

from toolbox.lane_detector import LaneDetector
from toolbox.color_detector import ColorDetector
from vpa_robot_interface.msg import WheelsEncoder

class DuckieRaceNode:
    def __init__(self):
        rospy.init_node('duckierace_node')
        self.robot_name = socket.gethostname()
        self.bridge = CvBridge()

        # ── 跟线相关 ──────────────────────────────────────────
        self.lane_detector  = LaneDetector()        # 默认：黄白线边界
        self.color_detector = ColorDetector()       # B键：直接跟黄线
        self.tof_valid_count = 0
        self.auto_yellow_trigger_count = 0
        self.image_half_width = 160                 # 图像半宽，默认320/2
        self.last_center_x    = None
        self.last_error       = 0.0
        self.last_time        = rospy.Time.now()
        self.lost_detect      = False

        # PD参数（跟黄线用，黄白线模式用lane_detector内部逻辑）
        self.kp = rospy.get_param('~kp', 2.0)
        self.kd = rospy.get_param('~kd', 1.0)

        # B键：是否切换到跟黄线模式
        self.yellow_mode      = False
        self.yellow_last_seen = rospy.Time.now()
        self.auto_yellow      = False               # ToF自动触发黄线模式

        # ── 速度相关 ──────────────────────────────────────────
        self.speed      = rospy.get_param('~init_speed', 0.3)
        self.speed_step = 0.02
        self.max_speed  = 0.35
        self.min_speed  = 0.2

        # ── ToF 前向防碰撞 ────────────────────────────────────
        self.z_min       = 0
        self.z_max       = 1.5
        self.tof_range   = self.z_max + 0.1         # 安全默认值
        self.tof_status  = 0
        self.tof_warn    = False
        rospy.Subscriber('front_range_status', Int32, self.tof_status_callback, queue_size=1)
        rospy.Subscriber('front_range', Range, self.tof_callback, queue_size=1)

        # ── 刹车 ──────────────────────────────────────────────
        self.local_brake     = True
        self.global_brake    = True
        self.last_brake_sent = False
        self.in_fuel_zone    = False
        self.charging        = False
        rospy.Subscriber('local_brake', Bool, self.local_brake_callback, queue_size=1)
        rospy.Subscriber('/global_brake', Bool, self.global_brake_callback, queue_size=1)
        rospy.Subscriber('in_fuel_zone', Bool, self.fuel_callback, queue_size=1)

        # ── 手柄 ──────────────────────────────────────────────
        self.last_joy_time = 0
        rospy.Subscriber('joy', Joy, self.joy_callback, queue_size=1)

        # ── 发布者 ────────────────────────────────────────────
        self.cmd_pub   = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.brake_pub = rospy.Publisher(f'/{self.robot_name}/local_brake', Bool, queue_size=1)

        # ── 摄像头 ────────────────────────────────────────────
        rospy.Subscriber('robot_cam/image_raw', Image, self.image_callback, queue_size=1)

        rospy.loginfo(f"[DuckieRace] Node initialized. Robot: {self.robot_name}, Init speed: {self.speed:.2f}")


    # ── ToF 回调 ──────────────────────────────────────────────
    def tof_status_callback(self, msg):
        if msg.data == 9:
            self.tof_valid_count = min(self.tof_valid_count + 1, 3)
        else:
            self.tof_valid_count = 0
        self.tof_status = 9 if self.tof_valid_count >= 3 else 0

    def tof_callback(self, msg):
        if self.tof_status == 9:
            self.tof_range = np.clip(msg.range - 0.04, self.z_min, self.z_max)
        else:
            self.tof_range = self.z_max + 0.1  # 无效读数视为安全

    # ── 刹车回调 ──────────────────────────────────────────────
    def local_brake_callback(self, msg):
        self.local_brake = msg.data

    def global_brake_callback(self, msg):
        self.global_brake = msg.data

    def fuel_callback(self, msg):
        self.in_fuel_zone = msg.data

    # ── 手柄回调 ──────────────────────────────────────────────
    def joy_callback(self, msg):
        now = time.time()
        if now - self.last_joy_time < 0.6:
            return
        self.last_joy_time = now

        # L1 加速
        if msg.buttons[4]:
            self.speed = min(self.speed + self.speed_step, self.max_speed)
            rospy.loginfo(f"[DuckieRace] Speed increased to: {self.speed:.2f}")

        # R1 减速
        if msg.buttons[5]:
            self.speed = max(self.speed - self.speed_step, self.min_speed)
            rospy.loginfo(f"[DuckieRace] Speed decreased to: {self.speed:.2f}")

        # X键 刹车控制
        if msg.buttons[2] and not self.last_brake_sent:
            if not self.in_fuel_zone:
                self.brake_pub.publish(Bool(data=False))
                self.last_brake_sent = True
                rospy.loginfo(f"[DuckieRace] Brake released")
            else:
                if msg.buttons[1]:  # 燃料区内需同时按B
                    self.brake_pub.publish(Bool(data=False))
                    self.last_brake_sent = True
                    rospy.loginfo(f"[DuckieRace] Brake released in fuel zone")
        elif msg.buttons[2] and self.last_brake_sent:
            if self.in_fuel_zone:
                self.brake_pub.publish(Bool(data=True))
                self.last_brake_sent = False
                rospy.loginfo(f"[DuckieRace] Brake engaged in fuel zone")

        # Y键 充电
        self.charging = bool(msg.buttons[3]) and self.in_fuel_zone

        # B键 切换黄线模式
        self.yellow_mode = bool(msg.buttons[1])

    # ── 图像回调：核心跟线逻辑 ────────────────────────────────
    def image_callback(self, msg):
        rospy.loginfo_throttle(2.0, f"[Mode] {'黄线模式' if (self.auto_yellow or self.yellow_mode) else '黄白线边界模式'}")

        # ToF 紧急停车
        if self.tof_range < 0.1:
            if not self.tof_warn:
                rospy.logwarn("[DuckieRace] Obstacle too close! Stopping.")
                self.tof_warn = True
            self.publish_cmd(0.0, 0.0)
            return
        else:
            if self.tof_warn:
                rospy.loginfo("[DuckieRace] Obstacle cleared.")
                self.tof_warn = False

        # 刹车检查
        if self.local_brake or self.global_brake:
            self.publish_cmd(0.0, 0.0)
            return

        # 图像转换
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(f"[DuckieRace] Image conversion error: {e}")
            return

        # ToF 自动切黄线（滞后逻辑，触发容易退出难）
        # if self.tof_range < 0.6:
        #     self.auto_yellow = True
        # elif self.tof_range > 1.0:
        #     self.auto_yellow = False
        # self.auto_yellow = self.tof_range < 0.5
        if self.tof_range <  0.7 :
            self.auto_yellow_trigger_count += 1
            if self.auto_yellow_trigger_count >= 3:
                self.auto_yellow = True
        else:
            self.auto_yellow_trigger_count = 0
            self.auto_yellow = False
        rospy.loginfo_throttle(0.5, f"[ToF] tof_range={self.tof_range:.3f} auto_yellow={self.auto_yellow} yellow_mode={self.yellow_mode}")

        if self.auto_yellow or self.yellow_mode:
            self._follow_yellow_line(bgr)
        else:
            self._follow_lane_boundaries(bgr)

    def _follow_lane_boundaries(self, bgr):
        """原程序跟线方式：检测黄线左边界+白线右边界，走中间"""
        result = self.lane_detector.image_process(bgr)

        if result is None or len(result) < 2:
            rospy.logwarn_once("[DuckieRace] Lane boundaries not detected.")
            self.publish_cmd(0.0, 0.0)
            return

        left_boundary  = result[0]
        right_boundary = result[1]
        lane_center_x  = (left_boundary + right_boundary) / 2

        # 平滑滤波
        if self.last_center_x is None:
            self.last_center_x = lane_center_x

        if abs(self.last_center_x - lane_center_x) <= 80:
            lane_center_x  = 0.8 * lane_center_x + 0.2 * self.last_center_x
            self.last_center_x = lane_center_x

            error     = lane_center_x - self.image_half_width
            now       = rospy.Time.now()
            dt        = (now - self.last_time).to_sec()
            derror    = (error - self.last_error) / dt if dt > 0 else 0.0
            angular_z = -(self.kp * error + self.kd * derror) / self.image_half_width
            angular_z = np.clip(angular_z, -2.0, 2.0)

            self.last_error = error
            self.last_time  = now
            self.publish_cmd(self.speed, angular_z)
        else:
            # 变化太大，保持原速直行
            self.publish_cmd(self.speed, 0.0)

    def _follow_yellow_line(self, bgr):
        """B键/ToF自动模式：直接跟黄线中心（ColorDetector方式）"""
        mask = self.color_detector.get_mask(bgr, 'yellow')
        h, w = mask.shape
        y_start = int(h * 0.6)
        y_end   = int(h * 0.8)
        roi     = mask[y_start:y_end, :]
        indices = np.where(roi > 0)[1]  # 取列坐标

        if len(indices) == 0:
            if not self.lost_detect:
                rospy.logwarn("[DuckieRace] Yellow line not detected!")
                self.lost_detect = True
            # 丢线时小角度左转找线
            self.publish_cmd(self.speed * 0.5, 0.5)
            return

        self.lost_detect = False
        avg_x    = int(np.mean(indices))
        center_x = w // 2
        offset   = int(w * 0.3)                         # 向左偏10%宽度
        error    = (avg_x - (center_x + offset)) / center_x

        now    = rospy.Time.now()
        dt     = (now - self.last_time).to_sec()
        derror = (error - self.last_error) / dt if dt > 0 else 0.0

        angular_z = -self.kp * error - self.kd * derror
        angular_z = np.clip(angular_z, -2.0, 2.0)

        self.last_error = error
        self.last_time  = now
        self.publish_cmd(self.speed, angular_z)

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = DuckieRaceNode()
    node.run()