#!/usr/bin/env python3
import sys
import time

import cv2
import rospy
import yaml
from cv_bridge import CvBridge
from dt_apriltags import Detector
from sensor_msgs.msg import Image


bridge = CvBridge()
images = {}


def in_zone(center, zone):
    x, y = center
    return zone["x_min"] <= x <= zone["x_max"] and zone["y_min"] <= y <= zone["y_max"]


def mask_ignore(gray, zones):
    out = gray.copy()
    for z in zones or []:
        cv2.rectangle(
            out,
            (int(z["x_min"]), int(z["y_min"])),
            (int(z["x_max"]), int(z["y_max"])),
            0,
            -1,
        )
    return out


def make_cb(topic):
    def cb(msg):
        images[topic] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    return cb


def main():
    rospy.init_node("codex_wait_fiona_fuel", anonymous=True)
    detector = Detector(families="tag36h11")
    configs = []

    for name, zone_path in [
        ("cam1", "/home/nanocar/catkin_ws/src/vpa_duckierace/config/zones_cam1.yaml"),
        ("cam2", "/home/nanocar/catkin_ws/src/vpa_duckierace/config/zones_cam2.yaml"),
    ]:
        with open(zone_path, "r") as f:
            cfg = yaml.safe_load(f)
        configs.append((name, cfg))
        rospy.Subscriber(cfg["image_topic"], Image, make_cb(cfg["image_topic"]), queue_size=1)

    deadline = time.time() + 180.0
    last_print = 0.0
    while not rospy.is_shutdown() and time.time() < deadline:
        seen = []
        for name, cfg in configs:
            img = images.get(cfg["image_topic"])
            if img is None:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray = mask_ignore(gray, cfg.get("ignore_zones", []))
            for det in detector.detect(gray):
                if int(det.tag_id) != 10:
                    continue
                center = tuple(float(v) for v in det.center)
                fuel = in_zone(center, cfg["fuel_zone"])
                seen.append((name, center, fuel))
                if fuel:
                    print(
                        f"FOUND_IN_FUEL {name} tag10 center=({center[0]:.1f},{center[1]:.1f})",
                        flush=True,
                    )
                    return 0

        now = time.time()
        if now - last_print > 5.0:
            if seen:
                parts = [
                    f"{name}:({center[0]:.1f},{center[1]:.1f}) in_fuel={fuel}"
                    for name, center, fuel in seen
                ]
                print("seen_tag10 " + " ".join(parts), flush=True)
            else:
                print("waiting_for_tag10_in_fuel", flush=True)
            last_print = now
        time.sleep(0.2)

    print("TIMEOUT tag10 not in fuel", flush=True)
    return 2


if __name__ == "__main__":
    sys.exit(main())
