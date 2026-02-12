# RC Tank with ROS2 - project

## About
This project is an **extension and adaptation of an existing ROS 1–based system**, 
migrated and redesigned to work with ROS 2.
Its primary objective is to enable **remote control of an RC tank using ROS 2 Twist commands**,
while preserving compatibility with the original low-level motor control logic.

The system is designed around a distributed architecture:

* A Raspberry Pi runs the low-level motor interface and ROS 2 control nodes.

* A PC or laptop provides user input via keyboard or joystick and publishes motion commands.

Both devices must be connected to the same network. Once the ROS 2 nodes are running on
the Raspberry Pi and a teleoperation node is started on the PC, the user can control the 
tank using either input method.

[Youtube Video](https://youtu.be/keiiRF_RTXY)

---

## System Overview

### Laptop
- Generates motion commands
- Input methods:
  - Keyboard (`teleop_twist_keyboard`)
  - Joystick (`joy` + `teleop_twist_joy`)
- Publishes `/cmd_vel`

### Raspberry Pi
- Converts `/cmd_vel` → Ackermann commands
- Sends motor commands over UART
- Runs:
  - `wc_bridge` (Twist → Ackermann)
  - `wc_main` (Ackermann → UART)

ROS 2 handles communication between laptop and Raspberry Pi over the network.

There is also an UART test, which the user can run to see if it is configured correctly.

## Prequisites
1. Raspberry Pi with ROS2 Jazzy and enabled UART pins
2. User PC or Laptop with ROS2 Jazzy

## Building UART test
```bash
cd RC_Tank/Tank/uart_bridge
./waf confiugre
./waf build
./waf run     # Run test application
./waf stop    # Stop test application
```

## Packed UART Command Format

The command byte format is:

`SSHHTTMM`

- `MM` (move)
  - `0`: `CM_IDLE`
  - `1`: `CM_FULL_BRAKE`
  - `2`: `CM_FORWARD`
  - `3`: `CM_REVERSE`
- `TT` (turn)
  - `0`: `CT_STRAIGHT`
  - `1`: `CT_RIGHT`
  - `2`: `CT_LEFT`
- `HH` (hardness)
  - `0`: `CTH_FLOAT`
  - `1`: `CTH_BRAKE`
  - `2`: `CTH_IN_PLACE`
- `SS` (speed)
  - `0`: `S_SLOW`
  - `1`: `S_MEDIUM`
  - `2`: `S_FAST`

---

## Running the system

### Raspberry Pi 
1. Clone the repository
```bash
git clone https://github.com/boskomit/OSuRV_2025.git
cd RC_Tank/Tank/vezba06_kernel_space/ROS2/chassis_ws
```
2. Build the ROS2 workspace
```bash
source /opt/ros/humble/setup.bash
colcon build
```

3. Run ROS2 nodes (two terminals)

Terminal 1 - Twist to Ackermann bridge
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run wc_bridge twist_to_ackermann
```
Terminal 2 - Ackermann controller
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run wc_main simple_ackermann_controller
```
4. Run step 3 faster (one terminal)
```bash
chmod +x run_pi.sh
./run_pi.sh
```
**The Raspberry Pi is now ready and waiting for `/cmd_vel`**

## Laptop - User input

### Keyboard control
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
You can now use the controls shown in the terminal.

The tank has 3 speed levels:
1. S_SLOW while (speed <= 33)
2. S_MEDIUM while (speed <= 66)
3. S_FAST 
   
Reverse speed is limited to S_MEDIUM

### Joystick control
1. Start joystick driver
```bash
source /opt/ros/jazzy/setup.bash
ros2 run joy joy_node
```
2. Start joystick ---> Twist node
```bash
ros2 run teleop_twist_joy teleop_node \
  --ros-args --params-file ~/path_to_this_file/teleop_twist_joy.yaml
```
The file teleop_twist_joy.yaml can be placed anywhere on the laptop.
Its path must be provided explicitly.

Our YAML:
```yaml
teleop_twist_joy_node:
  ros__parameters:
    require_enable_button: false
    axis_linear.x: 1
    axis_angular.yaw: 0
    scale_linear.x: 100.0
    scale_angular.yaw: 0.5
```
You can now use the left analog stick to control the tank.

## Credits
[WackyIdeas](https://github.com/WackyIdeas) for the original ROS1 project and most 
of the files in this project
