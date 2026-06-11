#!/usr/bin/env python3
import rospy
import yaml
import time
from std_msgs.msg import Float32, Bool, String
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import os

FONT_STACK = ["Inter", "SF Pro Display", "Helvetica Neue", "Arial", "DejaVu Sans"]

MODE_COLORS = {
    "manual": "#8A837A",
    "conservative": "#3B82C4",
    "aggressive": "#C65A45",
    "cooperative": "#2F9E73",
    "adaptive": "#9B6BBD",
    "-": "#C7BFB6",
}

DRIVING_MODES = ["manual", "conservative", "aggressive", "cooperative", "adaptive"]
SPEED_MIN = 0.20
SPEED_MAX = 0.35

BG = "#F8F6F2"
PANEL = "#FFFEFC"
TEXT = "#26211C"
MUTED = "#766F66"
BORDER = "#E3DDD5"
FILL_TRACK = "#EAE4DC"
POWER = "#B96A2D"
SPEED = "#416E9F"
SUCCESS = "#2F9E73"
DANGER = "#C65A45"
BUTTON = "#FFFEFC"
BUTTON_HOVER = "#F1ECE5"
SIDEBAR = "#FCFAF7"
SHADOW = "#EDE6DD"

FIGSIZE = (12, 6)
AX_ADJUST = {"top": 0.90, "right": 0.70, "left": 0.04, "bottom": 0.08}

CONTROL_PANEL = {
    "shadow": (0.718, 0.392, 0.27, 0.54),
    "panel": (0.715, 0.40, 0.27, 0.54),
    "labels": (
        (0.745, 0.817, "Phase presets"),
        (0.745, 0.635, "Robot strategies"),
    ),
}

BUTTON_LAYOUT = {
    "toggle": (0.745, 0.86, 0.215, 0.06),
    "phase1": (0.745, 0.735, 0.095, 0.058),
    "phase2": (0.865, 0.735, 0.095, 0.058),
    "mode_x": 0.745,
    "mode_y": 0.56,
    "mode_step": 0.085,
    "mode_width": 0.215,
    "mode_height": 0.055,
}

TABLE_HEADERS = (
    (0.05, "Robot"),
    (0.23, "Power"),
    (0.48, "Speed"),
    (0.71, "Brake"),
    (0.79, "Fuel"),
    (0.91, "Laps"),
)

ROW_TOP = 0.625
ROW_BOTTOM = 0.26
ROW_MAX_SPACING = 0.22
ROBOT_CARD = (0.0, 0.96, 0.14)
BAR_WIDTH = 0.20
BAR_HEIGHT = 0.035

plt.rcParams["font.family"] = "sans-serif"
plt.rcParams["font.sans-serif"] = FONT_STACK
plt.rcParams["axes.titleweight"] = "regular"


class RaceGUI:
    def __init__(self):
        rospy.init_node("race_gui_node")

        config = self.load_config()
        self.robots = config['robots']
        self.display_names = {k: v['display_name'] for k, v in self.robots.items()}
        self.robot_states = {name: self.default_robot_state() for name in self.robots}

        self.setup_ros()
        self.setup_race_state()
        self.setup_figure()
        self.setup_buttons()
        self.draw_controls_panel()
        self.anim = FuncAnimation(self.fig, self.update_gui, interval=200)
        plt.show()

    def load_config(self):
        config_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), 'config', 'robot_display_config.yaml')
        rospy.loginfo(f"Loading config from: {config_path}")
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    def default_robot_state(self):
        return {
            'power': 75.0,
            'speed': 0.0,
            'brake': True,
            'in_fuel': False,
            'laps1': 0,
            'laps2': 0,
            'mode': '-',
        }

    def setup_ros(self):
        for name in self.robots:
            rospy.Subscriber(f"/{name}/power_level", Float32, self.make_callback(name, 'power'))
            rospy.Subscriber(f"/{name}/speed_percent", Float32, self.make_callback(name, 'speed'))
            rospy.Subscriber(f"/{name}/local_brake", Bool, self.make_callback(name, 'brake'))
            rospy.Subscriber(f"/{name}/in_fuel_zone", Bool, self.make_callback(name, 'in_fuel'))
            rospy.Subscriber(f"/{name}/usb_cam_1/lap_count", Float32, self.make_callback(name, 'laps1'))
            rospy.Subscriber(f"/{name}/usb_cam_2/lap_count", Float32, self.make_callback(name, 'laps2'))
            rospy.Subscriber(f"/{name}/driving_mode", String, self.make_callback(name, 'mode'))

        self.global_brake_pub = rospy.Publisher("/global_brake", Bool, queue_size=1)
        self.mode_cmd_pubs = {
            name: rospy.Publisher(f"/{name}/set_driving_mode", String, queue_size=1)
            for name in self.robots
        }

    def setup_race_state(self):
        self.state = 'waiting'  # 'waiting', 'countdown', 'running'
        self.countdown = 5
        self.countdown_start = None
        self.start_time = None

    def setup_figure(self):
        self.fig, self.ax = plt.subplots(figsize=FIGSIZE, facecolor=BG)
        plt.subplots_adjust(**AX_ADJUST)

    def setup_buttons(self):
        self.toggle_ax = plt.axes(BUTTON_LAYOUT["toggle"])
        self.toggle_button = Button(self.toggle_ax, 'Start', color=BUTTON, hovercolor=BUTTON_HOVER)
        self.style_button(self.toggle_button, primary=True)
        self.toggle_button.on_clicked(self.toggle_game)

        self.phase1_ax = plt.axes(BUTTON_LAYOUT["phase1"])
        self.phase1_button = Button(self.phase1_ax, 'Phase 1\nAggressive',
                                    color="#FFF1F0", hovercolor="#FFE5E2")
        self.style_button(self.phase1_button, facecolor="#FFF7F5", edgecolor="#F2C8BF")
        self.phase1_button.on_clicked(self.set_phase1)

        self.phase2_ax = plt.axes(BUTTON_LAYOUT["phase2"])
        self.phase2_button = Button(self.phase2_ax, 'Phase 2\nCooperative',
                                    color="#ECFDF3", hovercolor="#DFF8EA")
        self.style_button(self.phase2_button, facecolor="#F4FCF7", edgecolor="#BFE5D0")
        self.phase2_button.on_clicked(self.set_phase2)

        self.mode_buttons = {}
        for idx, name in enumerate(self.robots):
            display_name = self.display_names[name]
            ax = plt.axes([
                BUTTON_LAYOUT["mode_x"],
                BUTTON_LAYOUT["mode_y"] - idx * BUTTON_LAYOUT["mode_step"],
                BUTTON_LAYOUT["mode_width"],
                BUTTON_LAYOUT["mode_height"],
            ])
            button = Button(ax, f"{display_name}: mode", color=BUTTON, hovercolor=BUTTON_HOVER)
            self.style_button(button)
            button.on_clicked(self.make_mode_button_cb(name))
            self.mode_buttons[name] = button

    def make_callback(self, robot_name, field):
        def callback(msg):
            self.robot_states[robot_name][field] = msg.data
        return callback

    def toggle_game(self, event):
        if self.state == 'waiting':
            self.countdown_start = time.time()
            self.state = 'countdown'
            self.toggle_button.label.set_text("Stop")
        elif self.state in ['countdown', 'running']:
            self.state = 'waiting'
            self.global_brake_pub.publish(Bool(data=True))
            self.toggle_button.label.set_text("Start")

    def set_phase1(self, event):
        self.broadcast_mode("aggressive")
        rospy.loginfo("[RaceGUI] Phase 1: all virtual drivers -> aggressive")

    def set_phase2(self, event):
        self.broadcast_mode("cooperative")
        rospy.loginfo("[RaceGUI] Phase 2: all virtual drivers -> cooperative")

    def broadcast_mode(self, mode_str):
        msg = String(data=mode_str)
        for pub in self.mode_cmd_pubs.values():
            pub.publish(msg)

    def make_mode_button_cb(self, robot_name):
        def callback(event):
            current = str(self.robot_states[robot_name]['mode'])
            try:
                next_idx = (DRIVING_MODES.index(current) + 1) % len(DRIVING_MODES)
            except ValueError:
                next_idx = 0
            mode_str = DRIVING_MODES[next_idx]
            self.mode_cmd_pubs[robot_name].publish(String(data=mode_str))
            self.robot_states[robot_name]['mode'] = mode_str
            rospy.loginfo(f"[RaceGUI] {robot_name} virtual driver -> {mode_str}")
        return callback

    def current_status(self):
        if self.state == 'waiting':
            return "Ready", "Waiting for race start", "STOPPED", MUTED

        if self.state == 'countdown':
            seconds_left = int(self.countdown - (time.time() - self.countdown_start))
            if seconds_left <= 0:
                self.start_time = time.time()
                self.state = 'running'
                self.global_brake_pub.publish(Bool(data=False))
            return f"{max(seconds_left, 0)}s", "Race starting", "COUNTDOWN", POWER

        elapsed = time.time() - self.start_time
        return f"{int(elapsed)}s", "Race running", "LIVE", SUCCESS

    def speed_fraction(self, speed):
        return self.clamp((speed - SPEED_MIN) / (SPEED_MAX - SPEED_MIN))

    def total_laps(self, state):
        return state['laps1'] + state['laps2']

    @staticmethod
    def clamp(value, low=0.0, high=1.0):
        return max(low, min(high, value))

    def style_button(self, button, facecolor=BUTTON, edgecolor=BORDER, primary=False):
        button.label.set_fontsize(9)
        button.label.set_color(TEXT)
        button.ax.set_facecolor(facecolor)
        button.ax.set_zorder(3)
        button.ax.patch.set_alpha(0.0)
        bbox = button.ax.get_position()
        pad = 0.004
        self.fig.patches.append(patches.FancyBboxPatch(
            (bbox.x0 - pad, bbox.y0 - pad), bbox.width + 2 * pad, bbox.height + 2 * pad,
            boxstyle="round,pad=0.006,rounding_size=0.009",
            linewidth=1.1, edgecolor=edgecolor,
            facecolor=facecolor,
            transform=self.fig.transFigure,
            zorder=1.5))
        if primary:
            button.label.set_fontsize(10)
            button.label.set_fontweight('bold')
        for spine in button.ax.spines.values():
            spine.set_visible(False)

    def draw_progress(self, x, y, width, height, frac, color):
        frac = self.clamp(frac)
        self.ax.add_patch(patches.FancyBboxPatch(
            (x, y - height / 2), width, height,
            boxstyle="round,pad=0.003,rounding_size=0.006",
            linewidth=0, facecolor=FILL_TRACK,
            transform=self.ax.transAxes))
        if frac > 0.005:
            self.ax.add_patch(patches.FancyBboxPatch(
                (x, y - height / 2), width * frac, height,
                boxstyle="round,pad=0.003,rounding_size=0.006",
                linewidth=0, facecolor=color,
                transform=self.ax.transAxes))

    def draw_pill(self, x, y, text, color, width=0.08):
        self.ax.add_patch(patches.FancyBboxPatch(
            (x, y - 0.018), width, 0.036,
            boxstyle="round,pad=0.006,rounding_size=0.008",
            linewidth=0, facecolor=color,
            alpha=0.12, transform=self.ax.transAxes))
        self.ax.text(x + width / 2, y, text,
                     fontsize=7.5, color=color, fontweight='bold',
                     ha='center', va='center',
                     transform=self.ax.transAxes)

    def draw_controls_panel(self):
        shadow_x, shadow_y, shadow_w, shadow_h = CONTROL_PANEL["shadow"]
        self.fig.patches.append(patches.FancyBboxPatch(
            (shadow_x, shadow_y), shadow_w, shadow_h,
            boxstyle="round,pad=0.012,rounding_size=0.012",
            linewidth=0, facecolor=SHADOW,
            alpha=0.65,
            transform=self.fig.transFigure,
            zorder=0.5))
        panel_x, panel_y, panel_w, panel_h = CONTROL_PANEL["panel"]
        self.fig.patches.append(patches.FancyBboxPatch(
            (panel_x, panel_y), panel_w, panel_h,
            boxstyle="round,pad=0.012,rounding_size=0.012",
            linewidth=1, edgecolor=BORDER, facecolor=SIDEBAR,
            transform=self.fig.transFigure,
            zorder=1.0))
        for x, y, label in CONTROL_PANEL["labels"]:
            self.fig.text(x, y, label, fontsize=9, color=MUTED, fontweight='bold')

    def update_gui(self, frame):
        self.ax.clear()
        self.ax.axis('off')
        self.ax.set_facecolor(BG)
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(0, 1)

        title, subtitle, status_text, status_color = self.current_status()
        self.draw_header(title, subtitle, status_text, status_color)
        self.draw_table_header()
        self.draw_robot_rows()
        self.draw_total_laps()

    def draw_header(self, title, subtitle, status_text, status_color):
        self.ax.text(0.02, 0.955, "DuckieRace Control",
                     fontsize=11, color=MUTED, fontweight='bold',
                     transform=self.ax.transAxes)
        self.ax.text(0.02, 0.865, title,
                     fontsize=38, color=TEXT, fontweight='bold',
                     transform=self.ax.transAxes)
        self.ax.text(0.02, 0.80, subtitle,
                     fontsize=10.5, color=MUTED,
                     transform=self.ax.transAxes)
        self.draw_pill(0.205, 0.803, status_text, status_color, width=0.095)

    def draw_table_header(self):
        for x, label in TABLE_HEADERS:
            self.ax.text(x, 0.715, label, fontsize=8.5, color=MUTED,
                         transform=self.ax.transAxes)

    def draw_robot_rows(self):
        spacing = self.row_spacing()
        for idx, (robot_name, state) in enumerate(self.robot_states.items()):
            self.draw_robot_row(robot_name, state, ROW_TOP - idx * spacing)

    def row_spacing(self):
        if len(self.robots) == 1:
            return 0.0
        return min(ROW_MAX_SPACING, (ROW_TOP - ROW_BOTTOM) / (len(self.robots) - 1))

    def draw_robot_row(self, robot_name, state, y):
        display_name = self.display_names[robot_name]
        mode_str = str(state['mode']) if state['mode'] else '-'
        mode_color = MODE_COLORS.get(mode_str, MODE_COLORS['-'])
        self.update_mode_button(robot_name, display_name, mode_str)
        self.draw_robot_card(y)

        self.ax.text(0.02, y + 0.018, display_name,
                     fontsize=13.5, fontweight='bold', color=TEXT,
                     transform=self.ax.transAxes,
                     verticalalignment='center')
        self.draw_pill(0.02, y - 0.032, mode_str, mode_color, width=0.11)
        self.draw_metric_values(state, y)
        self.draw_progress(0.23, y, BAR_WIDTH, BAR_HEIGHT,
                           self.clamp(state['power'] / 100.0), POWER)
        self.draw_progress(0.48, y, BAR_WIDTH, BAR_HEIGHT,
                           self.speed_fraction(state['speed']), SPEED)
        self.draw_brake_indicator(state, y)
        self.draw_fuel_indicator(state, y)
        self.ax.text(0.91, y, str(int(self.total_laps(state))),
                     fontsize=20, color=TEXT, fontweight='bold',
                     verticalalignment='center',
                     transform=self.ax.transAxes)

    def update_mode_button(self, robot_name, display_name, mode_str):
        if robot_name in self.mode_buttons:
            self.mode_buttons[robot_name].label.set_text(f"{display_name}: {mode_str}")

    def draw_robot_card(self, y):
        x, width, height = ROBOT_CARD
        self.ax.add_patch(patches.FancyBboxPatch(
            (x, y - 0.075), width, height,
            boxstyle="round,pad=0.012,rounding_size=0.010",
            linewidth=0, facecolor=PANEL,
            transform=self.ax.transAxes, zorder=0))

    def draw_metric_values(self, state, y):
        self.ax.text(0.23, y + 0.047, f"{int(state['power'])}%",
                     fontsize=8.5, color=MUTED,
                     transform=self.ax.transAxes,
                     verticalalignment='center')
        self.ax.text(0.48, y + 0.047,
                     f"{int(self.speed_fraction(state['speed']) * 100)}%",
                     fontsize=8.5, color=MUTED,
                     transform=self.ax.transAxes,
                     verticalalignment='center')

    def draw_brake_indicator(self, state, y):
        brake_color = DANGER if state['brake'] else SUCCESS
        self.draw_pill(0.70, y, "ON" if state['brake'] else "OFF",
                       brake_color, width=0.055)

    def draw_fuel_indicator(self, state, y):
        if state['in_fuel']:
            self.draw_pill(0.78, y, "IN FUEL", SUCCESS, width=0.085)
        else:
            self.draw_pill(0.78, y, "CLEAR", MUTED, width=0.075)

    def draw_total_laps(self):
        if self.state == 'running':
            total = sum(self.total_laps(s) for s in self.robot_states.values())
            self.ax.text(0.02, 0.07, f"Total laps: {int(total)}",
                         fontsize=12.5, ha='left', color=MUTED,
                         transform=self.ax.transAxes)


if __name__ == '__main__':
    RaceGUI()
