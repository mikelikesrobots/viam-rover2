# Viam Rover 2 ROS 2 Control

ROS 2 packages, configuration, and firmware for controlling a Viam Rover 2 robot on a Radxa X4 board.

Note that the Radxa X4 board contains an Intel N100 chip _and_ an RP2040 chip, commonly found on Raspberry Pi boards. The RP2040 controls the 40-pin header, and communicates with the N100 chip using a UART link. Hence, this project contains custom firmware that must be flashed to the RP2040 as part of the project (done automatically during project build).

This project also uses ros2_control for motor control, robot_localization for sensor fusion, RTAB-Map for SLAM, and Nav2 for autonomous navigation.

## Why Intel Hardware?

Two Intel components are central to this project:

**Radxa X4 (Intel N100)** - Gives full compatibility with standard Ubuntu and ROS 2 packages The N100's efficiency (6 W TDP) suits a battery-powered rover, while still providing enough headroom for SLAM and Nav2. The Radxa X4 pairs the N100 with a built-in RP2040, eliminating the need for a separate microcontroller board for GPIO and PWM.

**Intel RealSense D421** - A compact depth camera with a mature ROS 2 driver (`realsense2_camera`) that streams depth and infrared simultaneously. RTAB-Map consumes the depth stream directly for 3-D mapping; the infrared image provides a reliable texture source in low-light conditions where RGB cameras struggle.

Both components share the Intel software ecosystem: OpenVINO can accelerate future inference workloads on the same N100 hardware without additional accelerator cards.

## Features

- **ros2_control hardware interface** — C++ plugin that communicates with the RP2040 over serial; publishes encoder, IMU, and battery telemetry to ROS topics
- **RP2040 firmware** — C firmware for motor PWM, encoder interrupts, IMU (I²C), and battery ADC; binary telemetry protocol over UART
- **EKF sensor fusion** — robot_localization fuses wheel odometry and IMU into `/odometry/filtered`
- **RTAB-Map SLAM** — mapping and localization with an Intel RealSense D421 (depth + infrared)
- **Nav2 navigation** — path planning, obstacle avoidance, and behaviour-tree-driven goal navigation
- **Autonomous wandering** — integrated wandering application that explores a space using Nav2
- **Motor LUT feed-forward** — per-motor PWM → rad/s lookup table for accurate open-loop speed control
- **Calibration tools** — encoder tick calibration, motor sweep diagnostics, forward/rotation probes

## Hardware

| Component | Notes |
|-----------|-------|
| Viam Rover 2 chassis | Motor + encoder wiring per Viam pinout |
| Radxa X4 (main computer) | Runs ROS 2 Jazzy; UART4 → `/dev/ttyS4` to RP2040 |
| RP2040 microcontroller | Custom firmware in `firmware/rp2040/` |
| Intel RealSense D421 | USB; depth + infra1 topics used by RTAB-Map |
| RC filter (per encoder) | 100 Ω series resistor + 100 nF capacitor low-pass filter on each encoder signal line; 10 kΩ pull-up to 3.3 V. Assembled on breadboard. |

The purpose of this robot is to have an autonomous mapping robot with the lowest cost components, while also being simple to build. The Viam Rover 2.0 arrives pre-assembled, and I've tried to keep extra assembly to a minimum to add full autonomy capabilities to the robot.

## Wire Connections

For the Radxa X4 to interact with the robot components, the following wires must be connected from the board to the rover's 40-pin header. Each pin number given should be connected on both sides. See [this table](https://docs.radxa.com/en/x/x4/software/gpio) for the pin locations on the Radxa X4.

| GPIO Number | Pin Number | Reason        |
| ----------- | ---------- | ------------- |
| N/A         | 6          | Ground        |
| 13, 14, 15  | 33, 35, 37 | Left Motor    |
| 3, 7, 12    | 15, 29, 31 | Right Motor   |
| 28, 29      | 3, 5       | I2C Bus       |

Hence, the full list of pins to be directly connected is 3, 5, 6, 15, 29, 31, 33, 35, 37.

However, in addition to the direct connections above, extra connections are required for the encoders. These are sensors that track how fast the wheels are turning, but the signal lines are very noisy when the motors are running, so we use a pullup resistor and RC filter for each encoder to clean up the signal.

For each encoder, connect the pin from the robot into a separate row on the breadboard. The left encoder uses pin 38 (GPIO2), and the right encoder uses pin 40 (GPIO25); these are the input signals. Connect pin 1 (3.3V) and pin 9 (GND) from the Radxa to the breadboard. Insert wires into the Radxa X4 pins 38 and 40; these are the output signals.

For each of the signal lines, assemble the following circuit, using separate breadboard lines so the two circuits don't interact (the 3.3V row and the GND row can be connected to both circuits):

1. Connect a 10K resistor from 3.3V to the input signal line. This is the pullup resistor.
1. Connect a 100 Ohm resistor from the input signal to a new breadboard row. Insert the output signal into this row as well.
1. Connect a capacitor from this row to GND.

Without this circuit, the encoder counts become so noisy that they are unusable.

## Repository Structure

```
bringup/
  config/          YAML parameters — EKF, Nav2, controllers, RTAB-Map
  launch/          Launch files (see Launch Files section below)
description/
  ros2_control/    rover2.ros2_control.xacro — hardware params (ticks_per_rev etc.)
  urdf/            Robot URDF (joints, links, geometry)
firmware/
  rp2040/          RP2040 C firmware; build with CMake, flash with flash.sh
hardware/
  include/         C++ hardware interface headers
  src/             rover2_system.cpp, rp2040_bridge.cpp, rp2040_sensor_node.cpp
  test/            Hardware interface unit tests
tools/             Python diagnostic and calibration scripts
```

## Dependencies

The Radxa X4 must have a sufficient amount of storage space, a heat sink, and Ubuntu 24.04 installed.

Almost all software dependencies can be installed using the helper script provided by Intel in their [Getting Started guide](https://docs.openedgeplatform.intel.com/2026.0/edge-ai-suites/robotics-ai-suite/robotics/gsg_robot/index.html). The command is as follows:

```bash
wget https://raw.githubusercontent.com/open-edge-platform/edge-ai-suites/refs/heads/main/robotics-ai-suite/scripts/setup-robotics-jazzy.sh
chmod +x setup-robotics-jazzy.sh
export USE_PROXY=0
./setup-robotics-jazzy.sh
```

Any remaining dependencies will be installed by the `rosdep` command in the next section.

## Building

```bash
# Clone into your colcon workspace src/
cd ~/ros2_ws
git clone https://github.com/mikelikesrobots/viam-rover2.git src/viam_rover2

# Install ROS dependencies
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build
source install/setup.bash
```

---

## Calibration & Setup Guide

Follow these steps in order on first-time hardware setup. Each step builds on the previous one.

### Step 1 — Encoder Tick Calibration

The encoder tick count per wheel revolution (`left_ticks_per_rev`, `right_ticks_per_rev`) converts raw encoder pulses to metres. Incorrect values cause the robot to drift sideways or report wrong distances.

**Goal:** The robot moves in a straight line and the reported odometry distance matches the physical distance.

#### 1a. Straight-line correction

Launch the hardware stack with vision disabled:

```bash
ros2 launch viam_rover2 rover2_bringup.launch.py enable_vision:=false
```

In a second terminal, run the forward velocity probe:

```bash
python3 tools/forward_velocity_probe.py
```

Observe whether the robot tracks straight. If it veers left, the left wheel is running faster than the right (relative to what the controller expects) — increase `left_ticks_per_rev` slightly. If it veers right, increase `right_ticks_per_rev`. Edit the values in `description/ros2_control/rover2.ros2_control.xacro`:

```xml
<param name="left_ticks_per_rev">1570</param>
<param name="right_ticks_per_rev">1570</param>
```

Rebuild and relaunch after each change:

```bash
colcon build --packages-select viam_rover2
```

Repeat until the robot travels straight.

#### 1b. Distance calibration

With the robot going straight, run one forward probe and note the `odom_distance` value printed at the end. Physically measure the actual distance the robot travelled. Apply the correction formula to both wheels:

```
new_ticks_per_rev = old_ticks_per_rev * (odom_distance / actual_distance)
```

Update both values in the xacro file and rebuild.

---

### Step 2 — Motor LUT Calibration

The motor lookup table (LUT) maps commanded rad/s to the PWM value needed to achieve that speed, accounting for motor dead zones and non-linearity. The LUT is defined directly in `hardware/src/rover2_system.cpp` in `on_init()`.

**Goal:** Accurate feed-forward speed control so the controller doesn't have to fight large steady-state errors.

Place the robot on a box or stand so the wheels spin freely. Update the left and right encoder values in `tools/motor_diagnostic.py` so that speed can be accurately read from the wheel. Then, run the diagnostic script (no ROS required):

```bash
python3 tools/motor_diagnostic.py
```

The script sweeps PWM values, measures encoder-based rad/s for each step, and prints a LUT block ready to paste. Example output:

```
motor_lut_[0] = {  // left
    {0.1374, 0.0},
    {0.3, 1.94}, {0.4, 3.39}, ...
};
```

Copy both LUT blocks into `hardware/src/rover2_system.cpp`, replacing the existing `motor_lut_[0]` and `motor_lut_[1]` entries. Remove any entries where the `rad_s` value does not strictly increase from the previous entry. Rebuild:

```bash
colcon build --packages-select viam_rover2
```

---

### Step 3 — Mapping Mode

With calibrated encoders and motor LUT, build a map of the environment before running autonomously.

#### Launch in mapping mode

```bash
ros2 launch viam_rover2 wandering_mapper.launch.py mapping:=true
```

In a separate terminal, start the Foxglove bridge:

```bash
ros2 run foxglove_bridge foxglove_bridge
```

Connect Foxglove Studio to the robot (WebSocket, default port 8765).

#### Set up the Foxglove 3D panel

1. Add a **3D** panel.
2. In the panel settings, add the following topics:
   - `/map` — the occupancy grid being built
   - `/tf` — robot and sensor transforms
   - `/scan` — laser scan from the depth camera
   - `/odometry/filtered` — EKF pose (for a pose marker)
3. Set **Follow Mode** to *Follow pose* with the root as fixed frame and map as the display frame. This will ensure that waypoints are sent in the correct frame.

#### Drive and verify

Use Foxglove's **Teleop** panel (publish to `/cmd_vel`) to drive the robot around the space. In the panel settings, set **Publish rate** to `2` Hz, **Linear speed** to `0.5` m/s, and leave **Angular speed** at `1.0` rad/s.

- **Straight-line check:** drive forward and confirm the robot's path in the 3D panel is a straight line. If not, adjust the encoder tick values and repeat until the robot drives in a straight line.
- **Turn accuracy check:** make a 90° turn and compare the heading change shown in the map vs real life. This should be close enough that the robot's feedback mechanisms can compensate for the difference.

#### Completing the map

Restart the system, then drive around the entire space methodically with the Teleop panel until every area appears covered in the 3D panel. When satisfied with coverage, stop the launch with Ctrl+C — RTAB-Map saves its database automatically on shutdown. This saved map will be used for localisation in Step 4.

---

### Step 4 — Autonomous Wandering

With a saved map, the robot can localise itself and wander autonomously using Nav2 and the wandering application.

```bash
ros2 launch viam_rover2 wandering_mapper.launch.py
```

Once the 3D panel in Foxglove shows the saved map overlaid with the robot's position:

1. If the robot's initial pose is wrong, use Foxglove's **Publish** button to send a `geometry_msgs/PoseWithCovarianceStamped` to `/initialpose`.
2. The wandering application starts automatically after a short delay (~20 seconds from launch). The robot will begin navigating to random goals within the mapped area.

To stop wandering, press Ctrl+C on the launch terminal.

---

### Step 5 — Troubleshooting with Foxglove

#### Layout A — Log viewer

1. Open Foxglove Studio and connect to the robot.
2. Add a **Log** panel.
3. The panel will show all ROS log messages (`/rosout`). Filter by node name or severity to isolate problems.
   - Look for Error or Warn level from `Rover2SystemHardware` for serial/encoder issues.
   - Look for Nav2 nodes for navigation failures.

#### Layout B — Velocity comparison plot

This layout is useful for diagnosing speed control issues: whether the problem is in the navigation commands, the motor controller, or the encoder feedback.

1. Add a **Plot** panel.
2. Add the following series:

| Series | Topic path | Description |
|--------|-----------|-------------|
| Nav cmd linear.x | `/cmd_vel.linear.x` | Forward speed commanded by Nav2 (m/s) |
| Commanded left | `/rover2_base_controller/commanded_left.data` | Left wheel speed sent to hardware (rad/s) |
| Commanded right | `/rover2_base_controller/commanded_right.data` | Right wheel speed sent to hardware (rad/s) |
| Measured left | `/rover2_base_controller/measured_left.data` | Left wheel encoder speed (rad/s) |
| Measured right | `/rover2_base_controller/measured_right.data` | Right wheel encoder speed (rad/s) |

A large gap between *commanded* and *measured* on one wheel suggests encoder or motor wiring issues. If *commanded* and *measured* track each other poorly over time, the motor LUT may need re-calibration.

---

## Launch Files

| File | Purpose | Key arguments |
|------|---------|--------------|
| `rover2_bringup.launch.py` | Hardware interface, EKF, vision (RealSense + depth-to-scan), TwistStamper | `use_mock_hardware`, `enable_vision`, `enable_ekf`, `use_sim_time` |
| `rover2_control.launch.py` | ros2_control stack only (no vision) | `use_mock_hardware`, `serial_port`, `baud`, `ekf_config_file` |
| `wandering_mapper.launch.py` | Full autonomous system — rover bringup + RTAB-Map + Nav2 + wandering app | `mapping`, `use_rtabmap`, `use_rover2_control`, `use_mock_hardware` |
| `nav2_bringup.launch.py` | Nav2 nodes only (called by wandering_mapper) | `params_file`, `namespace`, `use_sim_time` |
| `rtabmap_d421.launch.py` | RTAB-Map with D421 depth + infra1 | `mapping`, `delete_db_on_start` |

**The most commonly used launch file is `wandering_mapper.launch.py`. Use `rover2_bringup.launch.py` during calibration when you want hardware up without the full autonomous stack.**

---

## Tools

All tools in `tools/` communicate either directly with the RP2040 over serial (no ROS) or via ROS topics. Run them from the repository root.

| Script | Requires ROS | Purpose |
|--------|-------------|---------|
| `tools/forward_velocity_probe.py` | Yes | Drive forward, compare odom/IMU/EKF distance and velocity |
| `tools/motor_diagnostic.py` | **No** | PWM sweep directly on RP2040; outputs motor LUT |
| `tools/rp2040_telemetry_monitor.py` | **No** | Live terminal display of encoder, IMU, and battery telemetry |

---

## Configuration Files

| File | Purpose |
|------|---------|
| `bringup/config/rover2_controllers.yaml` | ros2_control controller parameters (diff drive, joint state broadcaster) |
| `bringup/config/ekf.yaml` | robot_localization EKF sensor fusion config |
| `bringup/config/rover2_nav.param.yaml` | Nav2 parameters (planner, controller, costmaps, etc.) |
| `bringup/config/rtabmap_d421.yaml` | RTAB-Map parameters for the D421 camera |
| `bringup/config/navigate_to_pose_gentle_recovery_jazzy.xml` | Custom Nav2 behaviour tree |
| `description/ros2_control/rover2.ros2_control.xacro` | Hardware plugin parameters (`ticks_per_rev`, serial port, telemetry rates) |

The most commonly edited file is `description/ros2_control/rover2.ros2_control.xacro` during calibration.

---

## Firmware

The RP2040 firmware lives in `firmware/rp2040/`. It handles:

- **Motors** — PWM via `motor.c`
- **Encoders** — interrupt-driven tick counting via `encoder.c`
- **I2C sensors** — IMU and battery ADC via `i2c_sensors.c`
- **Protocol** — ASCII + binary UART protocol via `protocol.c` and `telemetry.c`

The firmware is built and flashed automatically as part of the `colcon build` step above. If you need to reflash manually (e.g. after modifying the firmware source), follow these steps:

Build and flash:

```bash
cd firmware/rp2040
mkdir build && cd build
cmake ..
make -j4
# Then with the RP2040 in BOOTSEL mode:
cd ..
bash flash.sh
```

---

## Mock Hardware

To test the ROS stack without a physical robot:

```bash
ros2 launch viam_rover2 rover2_bringup.launch.py use_mock_hardware:=true enable_vision:=false
```

Mock hardware mirrors velocity commands directly to state, so odometry will reflect commanded motion without real encoders.

---

## License

The main code is licensed under the MIT license (see [LICENSE](./LICENSE)).
Some files are derived from [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) and are covered by the [Apache 2.0 license](./ROS2_CONTROL_LICENSE).
