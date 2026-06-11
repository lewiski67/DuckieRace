#!/usr/bin/env python3
"""
Virtual Driver Node for DuckieRace
====================================
Publishes synthetic Joy messages to autonomously control one robot using a
selectable driving-behavior mode.  Designed to run on the main PC.

Behavior Modes
--------------
MANUAL       - Virtual driver is inactive; physical joystick is used instead.
CONSERVATIVE - Low speed, charges whenever inside the fuel zone.
AGGRESSIVE   - Max speed, charges only when power is critically low;
               switches to yellow line to overtake slow robots ahead.
COOPERATIVE  - Medium speed, yields at obstacles, shares the fuel zone fairly
               with the other robots.
ADAPTIVE     - Switches automatically between aggressive / conservative based
               on the current power level; rushes to the fuel zone when low.

ROS Interface
-------------
Subscribed topics (all absolute paths):
  /{robot_name}/power_level        Float32  Virtual power level (0-100 %)
  /{robot_name}/local_brake        Bool     Current local brake state
  /{robot_name}/in_fuel_zone       Bool     Whether robot is in the fuel zone
  /{robot_name}/in_charge_gate_zone Bool    Whether robot is at charge-gate entry
  /{robot_name}/front_range        Range    ToF distance to obstacle ahead
  /{robot_name}/set_driving_mode   String   Runtime mode-change command
  /{other_robot}/power_level       Float32  Peer robots' power (cooperative mode)
  /global_brake                    Bool     Race-level start / stop signal

Published topics:
  /{robot_name}/joy                Joy      Synthetic joystick commands
  /{robot_name}/driving_mode       String   Active mode name (latched)

Parameters
----------
~robot_name    str    default 'henry'         Robot ROS namespace
~driving_mode  str    default 'conservative'  Initial behavior mode
~all_robots    list   default [henry,fiona,dorie,luna]
~control_rate  float  default 2.0             Control-loop frequency [Hz]
"""

import rospy
from sensor_msgs.msg import Joy, Range
from std_msgs.msg import Bool, Float32, String
from enum import Enum


# ---------------------------------------------------------------------------
# Behaviour modes
# ---------------------------------------------------------------------------

class DrivingMode(Enum):
    MANUAL       = "manual"
    CONSERVATIVE = "conservative"
    AGGRESSIVE   = "aggressive"
    COOPERATIVE  = "cooperative"
    ADAPTIVE     = "adaptive"


# ---------------------------------------------------------------------------
# Joy button indices (PS4 / generic gamepad - same as duckierace.py)
# ---------------------------------------------------------------------------
BTN_B  = 1   # Yellow line / must hold to unlock brake inside fuel zone
BTN_X  = 2   # Brake toggle
BTN_Y  = 3   # Enable charging (while in fuel zone)
BTN_L1 = 4   # Increase speed by one step
BTN_R1 = 5   # Decrease speed by one step

# Speed parameters - keep in sync with duckierace.py
SPEED_START   = 0.25
SPEED_STEP    = 0.02
SPEED_MAX     = 0.35
SPEED_MIN     = 0.20
STEPS_TO_MAX  = int(round((SPEED_MAX - SPEED_START) / SPEED_STEP))  # 5
STEPS_TO_MIN  = -int(round((SPEED_START - SPEED_MIN) / SPEED_STEP)) # -2

# Power thresholds
POWER_CRITICAL    = 15.0   # % - charge now regardless of mode
POWER_CHARGE_FULL = 95.0   # % - stop charging above this
COOP_CHARGE_SELF  = 50.0   # % - cooperative robot charges below this
COOP_CHARGE_PEER  = 30.0   # % - cooperative robot yields if peer < this

# Obstacle distance [m]
OVERTAKE_DIST = 0.30        # activate yellow line to pass slow robot ahead
YIELD_DIST    = 0.25        # slow down (cooperative yield)


# ---------------------------------------------------------------------------
# Main node class
# ---------------------------------------------------------------------------

class VirtualDriver:
    def __init__(self):
        rospy.init_node("virtual_driver_node")

        self.robot_name  = rospy.get_param("~robot_name", "henry")
        mode_str         = rospy.get_param("~driving_mode", "conservative")
        all_robots       = rospy.get_param(
            "~all_robots", ["henry", "fiona", "dorie", "luna"])
        self.control_rate = float(rospy.get_param("~control_rate", 2.0))

        try:
            self.mode = DrivingMode(mode_str.strip().lower())
        except ValueError:
            rospy.logwarn(
                f"[VirtualDriver/{self.robot_name}] "
                f"Unknown mode '{mode_str}', falling back to 'conservative'.")
            self.mode = DrivingMode.CONSERVATIVE

        self.other_robots = [r for r in all_robots if r != self.robot_name]

        # ---- Race state -------------------------------------------------------
        self.power_level       = 75.0
        self.in_fuel_zone      = False
        self.in_charge_gate    = False
        self.global_brake      = True    # True = game not running
        self.tof_range         = 9.9     # metres - default "clear"
        self.other_power       = {r: 75.0 for r in self.other_robots}

        # ---- Internal driver state -------------------------------------------
        # Have we already sent the one-shot brake-release Joy message?
        self.brake_released    = False
        # Running estimate of the current speed step offset from SPEED_START.
        # +N means N * L1 presses sent, -N means N * R1 presses sent.
        self.speed_offset      = 0

        # ---- Publishers -------------------------------------------------------
        self.joy_pub  = rospy.Publisher(
            f"/{self.robot_name}/joy", Joy, queue_size=1)
        self.mode_pub = rospy.Publisher(
            f"/{self.robot_name}/driving_mode", String, queue_size=1, latch=True)

        # ---- Subscribers ------------------------------------------------------
        rospy.Subscriber(f"/{self.robot_name}/power_level",
                         Float32, self._cb_power)
        rospy.Subscriber(f"/{self.robot_name}/local_brake",
                         Bool,    self._cb_local_brake)
        rospy.Subscriber(f"/{self.robot_name}/in_fuel_zone",
                         Bool,    self._cb_fuel_zone)
        rospy.Subscriber(f"/{self.robot_name}/in_charge_gate_zone",
                         Bool,    self._cb_charge_gate)
        rospy.Subscriber(f"/{self.robot_name}/front_range",
                         Range,   self._cb_tof)
        rospy.Subscriber(f"/{self.robot_name}/set_driving_mode",
                         String,  self._cb_set_mode)
        rospy.Subscriber("/global_brake",
                         Bool,    self._cb_global_brake)

        for robot in self.other_robots:
            rospy.Subscriber(f"/{robot}/power_level", Float32,
                             self._make_power_cb(robot))

        # ---- Control timer ----------------------------------------------------
        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._control_loop)

        rospy.loginfo(
            f"[VirtualDriver/{self.robot_name}] "
            f"Started in '{self.mode.value}' mode at {self.control_rate} Hz")
        self.mode_pub.publish(String(data=self.mode.value))

    # ==========================================================================
    # Callbacks
    # ==========================================================================

    def _cb_power(self, msg: Float32):
        self.power_level = msg.data

    def _cb_local_brake(self, msg: Bool):
        pass  # reserved for future closed-loop brake logic

    def _cb_fuel_zone(self, msg: Bool):
        self.in_fuel_zone = msg.data

    def _cb_charge_gate(self, msg: Bool):
        self.in_charge_gate = msg.data

    def _cb_tof(self, msg: Range):
        self.tof_range = msg.range

    def _cb_global_brake(self, msg: Bool):
        if msg.data and not self.global_brake:
            # Game just stopped - reset so the next start performs brake release
            self.brake_released = False
            self.speed_offset   = 0
        self.global_brake = msg.data

    def _cb_set_mode(self, msg: String):
        try:
            new_mode = DrivingMode(msg.data.strip().lower())
        except ValueError:
            rospy.logwarn(
                f"[VirtualDriver/{self.robot_name}] "
                f"Unknown mode '{msg.data}'. "
                f"Valid: {[m.value for m in DrivingMode]}")
            return

        if new_mode != self.mode:
            self.mode           = new_mode
            self.brake_released = False   # re-release brake on mode change
            self.speed_offset   = 0
            rospy.loginfo(
                f"[VirtualDriver/{self.robot_name}] Mode -> {self.mode.value}")
            self.mode_pub.publish(String(data=self.mode.value))

    def _make_power_cb(self, name: str):
        def cb(msg: Float32):
            self.other_power[name] = msg.data
        return cb

    # ==========================================================================
    # Joy message helpers
    # ==========================================================================

    @staticmethod
    def _make_joy(buttons=None, axes=None) -> Joy:
        msg          = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.buttons  = buttons if buttons is not None else [0] * 12
        msg.axes     = axes    if axes    is not None else [0.0] * 8
        return msg

    def _joy_press(self, *button_indices) -> Joy:
        """Return a Joy message with specified buttons pressed, all others 0."""
        buttons = [0] * 12
        for idx in button_indices:
            if 0 <= idx < len(buttons):
                buttons[idx] = 1
        return self._make_joy(buttons=buttons)

    def _release_brake_msg(self) -> Joy:
        """
        One-shot Joy message that releases the brake.
        Inside the fuel zone the B button must be held simultaneously
        (duckierace.py requires B+X to unlock inside the fuel zone).
        """
        if self.in_fuel_zone:
            return self._joy_press(BTN_X, BTN_B)
        return self._joy_press(BTN_X)

    def _step_speed(self, target_offset: int, buttons: list) -> list:
        """
        Add ONE L1 or R1 press toward target_offset if not already there.
        Modifies and returns buttons in-place; updates self.speed_offset.
        """
        if self.speed_offset < target_offset:
            buttons[BTN_L1]   = 1
            self.speed_offset += 1
        elif self.speed_offset > target_offset:
            buttons[BTN_R1]   = 1
            self.speed_offset -= 1
        return buttons

    # ==========================================================================
    # Behavior implementations
    # ==========================================================================

    def _decide_conservative(self) -> Joy:
        """
        Low speed, always charge while in the fuel zone.

        Strategy rationale:
        - Minimises power consumption (keeps speed at minimum).
        - Recharges whenever an opportunity arises (fuel zone entry).
        - Never takes risky overtake manoeuvres.
        """
        buttons = [0] * 12

        # Nudge speed toward minimum
        self._step_speed(STEPS_TO_MIN, buttons)

        # Charge whenever possible
        if self.in_fuel_zone and self.power_level < POWER_CHARGE_FULL:
            buttons[BTN_Y] = 1

        return self._make_joy(buttons=buttons)

    def _decide_aggressive(self) -> Joy:
        """
        Max speed, charge only when critically low, overtake with yellow line.

        Strategy rationale:
        - Maximises lap throughput at the cost of higher power consumption.
        - Accepts the risk of running out of charge in exchange for speed.
        - Uses the yellow (alternative) track to pass slower robots.
        """
        buttons = [0] * 12

        # Nudge speed toward maximum
        self._step_speed(STEPS_TO_MAX, buttons)

        # Emergency charging only
        if self.in_fuel_zone and self.power_level < POWER_CRITICAL:
            buttons[BTN_Y] = 1

        # Overtake: switch to yellow line when obstacle is dangerously close
        if self.tof_range < OVERTAKE_DIST:
            buttons[BTN_B] = 1

        return self._make_joy(buttons=buttons)

    def _decide_cooperative(self) -> Joy:
        """
        Medium speed, fair fuel-zone sharing, yield at merge obstacles.

        Strategy rationale:
        - Yields (yellow line detour) when a robot is close to avoid blocking.
        - Charges only when its own level is low AND peer robots are not
          critically in need - preventing a "queue" at the charging spot.
        - Keeps moderate speed to reduce risk of rear-end collisions.
        """
        buttons = [0] * 12

        # Stay at default speed
        self._step_speed(0, buttons)

        # Yield when obstacle is ahead (cooperative merge behaviour)
        if self.tof_range < YIELD_DIST:
            buttons[BTN_B] = 1

        # Cooperative charging decision
        peers_not_critical = all(
            p > COOP_CHARGE_PEER for p in self.other_power.values())
        needs_charge = (
            self.in_fuel_zone
            and self.power_level < COOP_CHARGE_SELF
            and self.power_level < POWER_CHARGE_FULL
            and (peers_not_critical or self.power_level < POWER_CRITICAL))

        if needs_charge:
            buttons[BTN_Y] = 1

        return self._make_joy(buttons=buttons)

    def _decide_adaptive(self) -> Joy:
        """
        Dynamically switches strategy based on current power level.

        Power > 60 % -> aggressive (harvest laps while battery is good)
        Power 20-60 % -> conservative (ease off, charge when possible)
        Power < 20 % -> emergency rush (yellow line toward fuel zone)
        """
        if self.power_level > 60.0:
            return self._decide_aggressive()

        if self.power_level > 20.0:
            return self._decide_conservative()

        # Emergency: low power - navigate aggressively toward fuel zone
        buttons = [0] * 12
        if not self.in_fuel_zone:
            # Use yellow track; it often leads through the charging area
            buttons[BTN_B] = 1
        # Charge the moment we arrive
        if self.in_fuel_zone and self.power_level < POWER_CHARGE_FULL:
            buttons[BTN_Y] = 1
        return self._make_joy(buttons=buttons)

    # ==========================================================================
    # Main control loop (called by ROS timer at self.control_rate Hz)
    # ==========================================================================

    def _control_loop(self, _event):
        # MANUAL mode: leave the physical joystick in full control
        if self.mode == DrivingMode.MANUAL:
            return

        # Game not running: do nothing (brake already engaged by race GUI)
        if self.global_brake:
            return

        # --- One-shot brake release at game start ----------------------------
        if not self.brake_released:
            joy_msg = self._release_brake_msg()
            self.joy_pub.publish(joy_msg)
            self.brake_released = True
            rospy.loginfo(
                f"[VirtualDriver/{self.robot_name}] Brake released for '{self.mode.value}' mode")
            # Give duckierace.py one cycle (debounce window) before next command
            return

        # --- Normal behaviour ------------------------------------------------
        dispatch = {
            DrivingMode.CONSERVATIVE: self._decide_conservative,
            DrivingMode.AGGRESSIVE:   self._decide_aggressive,
            DrivingMode.COOPERATIVE:  self._decide_cooperative,
            DrivingMode.ADAPTIVE:     self._decide_adaptive,
        }
        decide_fn = dispatch.get(self.mode)
        if decide_fn is None:
            return

        self.joy_pub.publish(decide_fn())


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        VirtualDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
