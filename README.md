# 6‑Axis Robotic Arm Simulation with Inverse Kinematics (ROS2 + RViz)

This ROS2 package simulates a robotic arm with **continuous joints** (base yaw, shoulder pitch, elbow pitch, wrist pitch, and two prismatic fingers).  
A robust **inverse kinematics (IK) solver** using a Jacobian matrix and damped least squares drives the arm so that the **claw centre** follows a 3D target moved interactively in RViz.  
The claw orientation is decoupled – after positioning, the wrist joint is set to point directly at the target.

## Features

- **Interactive 3D target** – drag a sphere in RViz (XY plane + Z‑axis); the arm follows in real time.
- **Sub‑millimetre precision** – position error < 0.0001 m.
- **Capture detection** – marker turns green when the claw centre is within 1 cm; motion stops.
- **Workspace limits** – out‑of‑reach or too‑close targets are ignored (arm freezes).
- **Continuous joints** – all revolute joints are `continuous` (no limits, full 360° rotation).
- **Calibration constants** – `CLAW_CENTER_DIST` and `CLAW_URDF_OFFSET` let you align the kinematic model with any URDF visual.

## Dependencies

- ROS2 (Humble / Iron / Rolling – any distribution)
- `rclcpp`, `sensor_msgs`, `visualization_msgs`, `interactive_markers`, `rviz2`
- Eigen3 (`sudo apt install libeigen3-dev`)
- colcon build system

## Building

Clone the package into your ROS2 workspace, then build:

```bash
cd ~/ros2_ws/src
git clone <repository_url> my_robot
cd ~/ros2_ws
colcon build --packages-select my_robot
source install/setup.bash
