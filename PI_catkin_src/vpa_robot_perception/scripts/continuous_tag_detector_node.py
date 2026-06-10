#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import socket
import sys
import math
import numpy as np

sys.path.insert(0, '/home/vpaadmin/catkin_ws/src/vpa_robot_perception/scripts')
from toolbox.tag_detector import AprilTagWrapper

def yaw_zyx_from_R(R):
    return math.atan2(R[1,0], R[0,0])

class ContinuousTagDetectorNode:
    def __init__(self):
        rospy.init_node("continuous_tag_detector_node")
        self.robot_name = socket.gethostname()
        self.bridge = CvBridge()
        self.detector = AprilTagWrapper(debug=False, tag_size=0.06)

        self.base_to_camera = np.array([
            [-0.25881905,  0.0,  0.96592583,  0.0585],
            [-0.96592583,  0.0, -0.25881905,  0.0   ],
            [ 0.0,        -1.0,  0.0,         0.0742],
            [ 0.0,         0.0,  0.0,         1.0   ]
        ])

        self.pose_pub = rospy.Publisher("tag_relative_pose", Pose2D, queue_size=1)
        self.tag_id_pub = rospy.Publisher("detected_tag_id", Int32, queue_size=1)
        rospy.Subscriber("robot_cam/image_raw", Image, self.image_cb, queue_size=1)
        rospy.loginfo("%s: ContinuousTagDetectorNode started.", self.robot_name)

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            return

        detections, _ = self.detector.detect(frame, valid_tag_lowbound=0, valid_tag_upbound=999)
        if not detections:
            return

        det = detections[0]
        if det.get('pose_R') is None or det.get('pose_t') is None:
            return

        self.tag_id_pub.publish(det['id'])

        R_tc = det['pose_R']
        R_ct = R_tc.T
        t_tc = det['pose_t'].reshape(3)

        R_bc = self.base_to_camera[:3, :3]
        t_bc = self.base_to_camera[:3, 3]

        R_bt = R_bc @ R_ct
        p_b  = t_bc + R_bc @ t_tc

        T_base_to_tag = np.eye(4)
        T_base_to_tag[:3, :3] = R_bt
        T_base_to_tag[:3, 3]  = p_b
        T_tag_to_base = np.linalg.inv(T_base_to_tag)

        R_ct_from_base = R_bc.T @ R_bt
        yaw_rad = yaw_zyx_from_R(R_ct_from_base)

        x = -float(T_tag_to_base[0,3])
        y = -float(T_tag_to_base[1,3])

        pose_msg = Pose2D()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.theta = yaw_rad
        self.pose_pub.publish(pose_msg)

if __name__ == "__main__":
    try:
        ContinuousTagDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
