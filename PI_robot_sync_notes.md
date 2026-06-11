# PI robot sync notes

These notes record the fiona-side changes that should be copied to other PI robots before running DuckieRace.

## Target files

- `vpa_robot_operation/launch/duckierace_start.launch`
- `vpa_robot_operation/scripts/duckierace.py`

## Launch changes

In `duckierace_start.launch`:

- Set `lane_kd` default to `0.4`.
- Do not start the physical joystick node:
  - `joy_node` using `/dev/input/js0` is commented out.
  - Control should come from the PC-side virtual joystick publishing to `/<robot>/joy`.

Reason: if the physical joystick node and the PC virtual joystick both publish to the same robot joy topic, the car can receive conflicting commands.

## Line follower changes

In `duckierace.py`:

- Use `self.kp = 3.0`.
- Use `self.h_row_ratio = 0.76`.
- Use a single scan row at `h_row_ratio` instead of multiple voting rows.
- Compute the line center with `np.mean(indices)`.
- Limit angular speed with:

```python
angular_speed = np.clip(angular_speed, -0.8, 0.8)
```

## ToF change

The ToF obstacle stop in `image_callback()` is disabled for DuckieRace:

```python
# ToF obstacle stopping disabled for DuckieRace; the front ToF is unreliable on this car.
```

Reason: fiona's front ToF was unreliable during DuckieRace testing and caused unwanted stopping.

## Runtime expectation

Each robot should be started with correct ROS networking for its own IP, for example:

```bash
export ROS_MASTER_URI=http://192.168.1.4:11311
export ROS_IP=<robot_pi_ip>
unset ROS_HOSTNAME
```

For fiona, the tested IP was `192.168.1.13`.
