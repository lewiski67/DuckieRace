#!/usr/bin/env python3
import rospy
import yaml
import time
from std_msgs.msg import Float32, Bool
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import os

class RaceGUI:
    def __init__(self):
        rospy.init_node("race_gui_node")
        self.last_detect = False
        # Load robot display config
        config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'robot_display_config.yaml')
        rospy.loginfo(f"Loading config from: {config_path}")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        self.robots = config['robots']
        self.display_names = {k: v['display_name'] for k, v in self.robots.items()}

        self.robot_states = {}
        for name in self.robots:
            self.robot_states[name] = {
                'power': 75.0,
                'speed': 0.0,
                'brake': True,
                'in_fuel': False,
                'laps1': 0,
                'laps2': 0
            }
            rospy.Subscriber(f"/{name}/power_level", Float32, self.make_callback(name, 'power'))
            rospy.Subscriber(f"/{name}/speed_percent", Float32, self.make_callback(name, 'speed'))
            rospy.Subscriber(f"/{name}/local_brake", Bool, self.make_callback(name, 'brake'))
            rospy.Subscriber(f"/{name}/in_fuel_zone", Bool, self.make_callback(name, 'in_fuel'))
            rospy.Subscriber(f"/{name}/usb_cam_1/lap_count", Float32, self.make_callback(name, 'laps1'))
            rospy.Subscriber(f"/{name}/usb_cam_2/lap_count", Float32, self.make_callback(name, 'laps2'))

        self.global_brake_pub = rospy.Publisher("/global_brake", Bool, queue_size=1)

        # GUI setup
        self.state = 'waiting'  # 'waiting', 'countdown', 'running'
        self.countdown = 5
        self.countdown_start = None
        self.start_time = None

        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        plt.subplots_adjust(top=0.8, right=0.75)
        self.anim = FuncAnimation(self.fig, self.update_gui, interval=200)

        # Start/Stop toggle button
        self.toggle_ax = plt.axes([0.8, 0.85, 0.15, 0.08])
        self.toggle_button = Button(self.toggle_ax, 'Start Game')
        self.toggle_button.on_clicked(self.toggle_game)

        plt.show()

    def make_callback(self, robot_name, field):
        def callback(msg):
            if field in ['power', 'speed','laps1', 'laps2']:
                self.robot_states[robot_name][field] = msg.data
            else:
                self.robot_states[robot_name][field] = msg.data
        return callback

    def toggle_game(self, event):
        if self.state in ['waiting', 'running']:
            if self.state == 'waiting':
                # Start game countdown
                self.countdown_start = time.time()
                self.state = 'countdown'
                self.toggle_button.label.set_text("Stop Game")
            elif self.state == 'running':
                # Stop game
                self.state = 'waiting'
                self.global_brake_pub.publish(Bool(data=True))
                self.toggle_button.label.set_text("Start Game")

    def update_gui(self, frame):
        self.ax.clear()
        self.ax.axis('off')

        # Title/Header
        if self.state == 'waiting':
            title = "Waiting to Start..."
        elif self.state == 'countdown':
            seconds_left = int(self.countdown - (time.time() - self.countdown_start))
            if seconds_left <= 0:
                self.start_time = time.time()
                self.state = 'running'
                self.global_brake_pub.publish(Bool(data=False))
            title = f"Race Starting In: {max(seconds_left, 0)}s"
        elif self.state == 'running':
            elapsed = time.time() - self.start_time
            title = f"Race Running: {int(elapsed)}s"
        self.ax.set_title(title, fontsize=20)

        # Robot display blocks
        spacing = 1.0 / (len(self.robots) + 1)
        for idx, (robot_name, state) in enumerate(self.robot_states.items()):
            y = 0.7 - idx * spacing

            display_name = self.display_names[robot_name]
            self.ax.text(0.05, y, display_name, fontsize=14)

            # Power bar
            bar_x1 = 0.25
            self.ax.add_patch(patches.Rectangle((bar_x1, y - 0.025), 0.2, 0.05, fill=False))
            self.ax.add_patch(patches.Rectangle(
                (bar_x1, y - 0.025), 0.2 * min(state['power'], 100.0) / 100.0, 0.05, color='orange'))
            self.ax.text(bar_x1 + 0.1, y + 0.035, f"Power: {int(state['power'])}%", fontsize=10, ha='center')

            # Speed bar (scaled between 0 and 0.4 → 0–100%)
            bar_x2 = 0.55
            self.ax.add_patch(patches.Rectangle((bar_x2, y - 0.025), 0.2, 0.05, fill=False))
            spd = (state['speed'] - 0.2)/0.15

            spd_per_to_display = max(0, min(100, 100 * spd))
            self.ax.add_patch(patches.Rectangle(
                (bar_x2, y - 0.025), 0.2 * spd_per_to_display / 100.0, 0.05, color='blue'))
            self.ax.text(bar_x2 + 0.1, y + 0.035, f"Speed: {int(spd_per_to_display)}%", fontsize=10, ha='center')

            # Brake indicator
            brake_color = 'red' if state['brake'] else 'green'
            self.ax.add_patch(patches.Circle((0.87, y), 0.015, color=brake_color))
            self.ax.text(0.89, y, "Brake", verticalalignment='center')

            # Fuel zone indicator
            if state['in_fuel']:
                self.ax.text(0.02, y-0.05, "OK to Fuel", fontsize=12, verticalalignment='center', color='green')
            else:
                self.ax.text(0.02, y-0.05, "NG to Fuel", fontsize=12, verticalalignment='center', color='red')

            # Lap counter
            self.ax.text(1.08, y, f"Laps: {state['laps1']+state['laps2']}", fontsize=14, verticalalignment='center')


if __name__ == '__main__':
    RaceGUI()
