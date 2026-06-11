# DuckieRace Demo

## DuckieRace

DuckieRace is a demonstration project based on Duckietown robots in the MiniCCAM Lab.

It is designed for:

- Public events and lab visits
- Student course projects
- Introductory experiments in autonomous driving and multi-robot interaction

This page provides a practical guide for operating the DuckieRace system, including:

- How to start the robots
- How to use the joystick
- How the race logic works (fuel zone, merge zone, charge gate)
- Basic troubleshooting

The goal of this document is operational clarity rather than technical depth. It is intended for new lab members or teaching assistants who need to run the demo reliably.

### Playground Setup

The DuckieRace system runs on a printed track sheet placed on a flat floor or large table. The setup consists of:

- One printed race track sheet
- Two overhead USB cameras
- Four Duckietown robots (DB19 platform)
- One main computer running the race control and camera manager
- Joystick controllers for manual interaction
- Local network

#### Track Sheet

The track sheet source file is available at:  
[ https://datashare.tu-dresden.de/index.php/f/161691964 ](https://datashare.tu-dresden.de/index.php/f/161691964)

The layout looks as follows:

[ ![FormulaDuckie.jpg](https://wiki.vlpz.vkw.tu-dresden.de/uploads/images/gallery/2026-03/scaled-1680-/formuladuckie.jpg) ](https://wiki.vlpz.vkw.tu-dresden.de/uploads/images/gallery/2026-03/formuladuckie.jpg)

The red track is the main racing loop. The yellow track provides alternative paths, allowing robots to detour to the charging area or overtake other robots.

**Important:** The sheet must lie flat. Avoid wrinkles, strong reflections, or uneven lighting, as these can affect line detection.

#### Camera Placement

Cameras may be mounted using tripods for portability. In the MiniCCAM lab, a fixed overhead camera structure is available.

An example camera placement is shown below:

[ ![camera_placement.png](https://wiki.vlpz.vkw.tu-dresden.de/uploads/images/gallery/2026-03/scaled-1680-/camera-placement.png) ](https://wiki.vlpz.vkw.tu-dresden.de/uploads/images/gallery/2026-03/camera-placement.png)

The key requirement is that each camera must clearly see the three calibration tags located near the yellow track on its side. These tags are used to define merge zones, fuel zones, and charge gate areas. After any change in camera position, recalibration must be performed.

#### Robots and Joysticks

Any DB19 robot (red chassis, dual-wheel drive) prepared for MiniCCAM can be used without additional software installation, as the required code is already deployed on the robot.

Before operation, check:

- The robot battery level
- The AprilTag on top of the robot is clean and clearly visible
- The joystick and its USB adapter are functioning

There is currently no fixed mapping between specific joysticks and robots. If unsure which joystick corresponds to which robot, unplug the adapter and test it on a PC using:

[ https://hardwaretester.com/gamepad ](https://hardwaretester.com/gamepad)

This allows verification of button indices and joystick functionality.

#### Local WiFi Network

The robots and the main control computer must be connected to the same WLAN network segment.

For reliable operation, all devices should be on the same local network to ensure stable ROS communication between:

- The main PC (running the camera manager and control nodes)
- The Duckietown robots

The most convenient setup is to use the dedicated MiniCCAM lab PC together with the lab router. For off-site demonstrations, bring both the PC and the router to the demo location and connect all robots to that router.

### ROS Package Setup

#### Robots

Each DB19 robot must have the following two ROS packages installed in its workspace:

- **vpa\_robot\_interface**   
  Branch: `db19_devel_wheelspeed_enhanced`   
  Repository: [ https://github.com/VPAMINICCAM/vpa\_robot\_interface/tree/db19\_devel\_wheelspeed\_enhanced ](https://github.com/VPAMINICCAM/vpa_robot_interface/tree/db19_devel_wheelspeed_enhanced)
- **vpa\_robot\_operation**   
  Branch: `duckierace`   
  Repository: [ https://github.com/YikaiZengTUD/vpa\_robot\_operation/tree/duckierace ](https://github.com/YikaiZengTUD/vpa_robot_operation/tree/duckierace)

**Important:** Make sure the correct branches are checked out. The system will not behave correctly if another branch is active.

##### Verify Branch and Update

On each robot, navigate to the catkin workspace (typically `~/catkin_ws/src`) and run:

```
cd ~/catkin_ws/src/vpa_robot_interface
git branch
git pull

cd ~/catkin_ws/src/vpa_robot_operation
git branch
git pull

```

The active branch is marked with an asterisk (\*). Ensure it matches the required branch listed above.

If necessary, switch to the correct branch:

```
git checkout db19_devel_wheelspeed_enhanced
git pull

git checkout duckierace
git pull

```

After updating, rebuild the workspace:

```
cd ~/catkin_ws
catkin_make
source devel/setup.bash

```

**Warning:** The `vpa_robot_operation` repository is currently hosted under a personal GitHub account and may be transferred or archived in the future. At the time of writing, the link above is valid. If the repository becomes unavailable, contact the MiniCCAM lab maintainers for the updated location.

#### Main PC

The main PC runs the race control and camera-side logic. The corresponding ROS package is hosted at:

[ https://gitlab.vlpz.vkw.tu-dresden.de/yikai/vpa\_duckierace ](https://gitlab.vlpz.vkw.tu-dresden.de/yikai/vpa_duckierace)

This repository is hosted on the internal VPA GitLab and is expected to remain available.

##### USB Cameras

Two USB cameras must be connected to the main PC.

First, verify that both cameras are detected by the operating system:

```
ls /dev/video*
```

You should see at least two video devices (for example `/dev/video0` and `/dev/video2`).

In `launch/race_camera.launch`, only the following parameter typically needs adjustment:

```
<param name="video_device" value="/dev/videoX" />
```

Update `/dev/videoX` to match the correct device index for each camera.

**Camera requirements:**

- Wide field of view to cover the needed
- Stable 15 FPS or higher
- Compatible with ROS `usb_cam` driver

No specific camera model is required.

**Intrinsic calibration is not required.** The system does not use AprilTag pose estimation. Only the pixel positions of detected tags are used to define zones. Therefore, camera calibration files are not critical for correct operation.

It does not matter which physical camera is assigned to camera 1 or camera 2, as long as zone calibration is performed consistently afterward.

### Robot Configuration

The main PC uses two YAML files in the `config/` folder to define which robots participate in the game and how they appear in the GUI.

#### 1) `robot_display_config.yaml` (GUI display only)

This file controls how robots are shown in the race GUI. It does **not** affect the robot behavior or tag detection.

Typical example:

```
robots:
  henry:
    display_name: ROBOT1
    color: dark green
  fiona:
    display_name: ROBOT2
    color: purple
  dorie:
    display_name: ROBOT3
    color: blue
  luna:
    display_name: ROBOT4
    color: white

```

This is mainly used for public demos. For example, if a visitor wants a custom name, you can change `display_name` and it will appear in the GUI.

#### 2) `tag_robot_map.yaml` (required)

This file defines the mapping between robot names and AprilTag IDs. It is **required** for correct operation.

Example:

```
henry: 6
vivian: 2
fiona: 10
dorie: 7
luna: 8

```

**Rules:**

- It must include every robot that will participate in the game.
- The key (left side) is the robot name used as the ROS namespace (e.g., `/henry/...`).
- The value (right side) is the AprilTag ID printed on the robot’s tag.
- Each robot must have a unique tag ID.

If this mapping is wrong or incomplete, the camera manager may publish to the wrong topics and the robots will not respond correctly (e.g., braking, fuel/charge gate detection, lap counting).

### Zone Calibration Procedure

Zone calibration must be performed whenever:

- The cameras are moved
- The track position changes
- Lighting conditions change significantly

The calibration process defines the fuel zone, merge zone, charge gate, and ignore zones automatically from three reference AprilTags per camera.

#### Step 1 – Start the Camera Nodes

On the main PC, launch the camera system:

```
roslaunch vpa_duckierace racecontrol_camera.launch

```

Make sure both camera topics are publishing:

```
rostopic list | grep image

```

#### Step 2 – Capture Calibration Images

It is recommended to use `rqt_image_view` to manually capture one image per camera.

```
rqt_image_view

```

Select:

- `/usb_cam_1/image_raw`
- `/usb_cam_2/image_raw`

For each camera:

- Ensure the three calibration tags are clearly visible.
- <span style="color: rgb(224, 62, 45);">**ONLY THESE 3 TAGS ARE IN PICTURE**</span>
- Ensure no robots are inside the field of view.
- Save the image.

Save the images as:

```
vpa_duckierace/test/image1.png
vpa_duckierace/test/image2.png

```

#### Alternative: Capture from Command Line

You may also capture images directly using:

```
rosrun image_view image_saver image:=/usb_cam_1/image_raw _filename_format:=image1.png

```

Then repeat for camera 2:

```
rosrun image_view image_saver image:=/usb_cam_2/image_raw _filename_format:=image2.png

```

Move the generated files into the `test/` folder of the `vpa_duckierace` package if necessary.

#### Important Notes

- Only the three calibration tags for that camera should be visible.
- Robots must not appear in the image.
- The tags must not move after calibration.

After saving both images, run the zone calibration script to automatically generate the updated `zones_cam1.yaml` and `zones_cam2.yaml` files.

```bash
cd ~/catkin_ws/src/vpa_duckierace
python scripts/auto_zone_cali.py
```

### Starting the Game

1. **Place the robots on the start positions.**  
   Position each robot on the designated starting fields of the track. Ensure they are aligned with the lane and not blocking each other.

2. **Launch the race control on the main PC:**```
   roslaunch vpa_duckierace start_racecontrol.launch

   ```
   
   1. Two preview windows will appear showing the automatically generated zones (fuel zone, merge zone, charge gate) for verification.
   2. Visually confirm that the zones align correctly with the track.
   3. Close the preview windows. The race control scripts will continue running in the background.
   ```

3. **Start the robot-side nodes.**  
   For each robot, SSH into the robot and run: ```
   roslaunch vpa_robot_operation duckierace_start.launch

   ```
   
   ```

4. **Start the race GUI on the main PC:**```
   rosrun vpa_duckierace race_gui.py

   ```
   
   ```

5. In the GUI, press the **Start Game** button.

6. The system will perform a 5-second countdown and then send a start signal to all robots.

7. **Important:** Even after the countdown, robots will not move until their local brake is released via the joystick.

8. The game can be stopped at any time by pressing the **Stop Game** button in the GUI.

### Joystick Control

Each robot is controlled locally using a joystick. The joystick does not directly drive the robot, but enables or modifies autonomous behavior.

#### Speed Control

- **L1** – Increase speed
- **R1** – Decrease speed

Speed changes in small steps. Increasing speed also increases virtual power consumption.

#### Brake Control (X Button)

The **X button** (button index 2) controls the local brake.

- Press **X** to release the brake and allow the robot to move.
- Press **X** again (while in fuel zone) to re-engage the brake.

**Important behavior:**

- Outside the fuel zone, the brake can always be released normally.
- Inside the fuel zone, the brake can only be released while holding the **B button**.
- If B is not held in the fuel zone, the robot will remain stopped.

#### Charging (Y Button)

- **Y** (button index 3) enables charging while the robot is inside the fuel zone.
- The robot must remain stopped in the fuel zone to charge.

#### Alternative Line Mode (B Button)

- **B** (button index 1) activates alternative line following (yellow track).
- When released, the robot returns to the default red track.

This is used for overtaking or detouring to the charging area.

#### Summary of Buttons

- L1 – Increase speed
- R1 – Decrease speed
- X – Release / toggle brake
- B – Yellow line mode (must hold in fuel zone to unlock brake)
- Y – Enable charging (only inside fuel zone)

### Race GUI

The race GUI provides a simple live dashboard for the DuckieRace demo and includes Start/Stop control for the game.

Start the GUI on the main PC with:

```
rosrun vpa_duckierace race_gui.py
```

#### What the GUI Shows

For each robot (as defined in `config/robot_display_config.yaml`), the GUI displays:

- **Robot name** (display name from the config file)
- **Power level** (orange bar, 0–100%)
- **Speed** (blue bar, displayed as a percentage)
- **Brake status** (indicator light: red = brake on, green = brake released)
- **Fuel zone permission** (text indicator: “OK to Fuel” / “NG to Fuel”)
- **Lap counter** (sum of lap counts reported from both cameras)

#### Start / Stop Button

The GUI includes a single control button in the upper right:

- **Start Game** – begins a 5-second countdown and then starts the race
- **Stop Game** – stops the race immediately

During the countdown and race, the GUI title bar shows the current state:

- **Waiting to Start...**
- **Race Starting In: 5s ... 0s**
- **Race Running: XXs** (elapsed time)

#### Important Notes

- The GUI publishes a global start/stop signal on `/global_brake`.
- Even after the game starts, a robot will not move unless its **local brake** is released via the joystick.
- Lap count is computed as: `usb_cam_1/lap_count + usb_cam_2/lap_count`.

### Demo Flow and Learning Goal

The DuckieRace demo is designed to be interactive and easy to understand for visitors, while also introducing a deeper research idea about coordination and cooperation in transportation systems.

#### Phase 1 – Free Play (No Coordination)

First, participants are invited to drive and interact with the robots freely. In this phase, everyone acts independently.

What typically happens:

- Robots block each other at merge areas
- The charging area becomes occupied and causes congestion
- Some robots run low on power and stop unexpectedly
- Overtaking and detours are used in an unplanned way

Run this phase for a fixed time window (recommended: **5 minutes**). At the end, record the **total lap count** (sum over all robots).

#### Phase 2 – Cooperative Play (Negotiation and Coordination)

Next, introduce the idea of **cooperative transportation**. Participants are asked to communicate and coordinate their actions to improve overall performance.

Example cooperative strategies:

- Negotiate who gets access to the charging area and when
- Avoid blocking at merge points by yielding intentionally
- Plan overtakes and detours to reduce conflicts
- Coordinate charging so that robots do not all stop at the same time

Run the cooperative phase for the same duration (again: **5 minutes**) and record the total lap count.

#### Key Message

The main message of this demo is that individual, selfish decisions often lead to congestion and reduced system performance, while communication and coordination can improve the outcome for everyone.

The success metric is simple and visible: **overall lap count across all robots**.